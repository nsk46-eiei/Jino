# FinalTLS_ETA.py  (FinalnoTLS + TLS delay ETA-aware, no green bias)
# -*- coding: utf-8 -*-

"""
========================================
SUMO Routing (with TLS delay ETA-aware, no green bias)
========================================

สารบัญ
  1) Imports & Bootstrap
  2) Parameters (ค่าตั้งค่า)
  3) Global Rolling Buffers
  4) Config Helpers
  5) Lane/Edge Utilities
  6) Build Edge Graph (nodes = edges)
  7) Cost Model (base weights only; no global TLS here)
  8) TLS ETA-aware utilities
  9) Prediction Helper (Method A)
 10) Main Loop
"""

# -------------------------------------------------------------------
# 1) IMPORTS & BOOTSTRAP
# -------------------------------------------------------------------
import os
import sys
import glob
import csv
import time
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
from typing import Dict, List, Optional, Set, Tuple
import warnings

warnings.filterwarnings("ignore", message=".*deprecated function getAllProgramLogics.*")

# ---- SUMO bootstrap ----
try:
    from sumolib import checkBinary
except Exception:
    raise RuntimeError("Install: pip install sumolib traci networkx")

if "SUMO_HOME" in os.environ:
    TOOLS = os.path.join(os.environ["SUMO_HOME"], "tools")
    if os.path.isdir(TOOLS):
        sys.path.append(TOOLS)

import traci  # type: ignore
import sumolib  # type: ignore
import networkx as nx  # type: ignore


# -------------------------------------------------------------------
# 2) PARAMETERS (ค่าตั้งค่า)
# -------------------------------------------------------------------
REROUTE_PERIOD: float = 1.0             # คาบเวลาการ reroute ต่อคัน (s)
NEAR_JUNCTION_SKIP_DIST: float = 20.0   # ใกล้จุดตัดให้ข้ามการ setRoute
DENSITY_ALPHA: float = 2                # ค่าถ่วงความหนาแน่น
MIN_SPEED_FRACTION: float = 0.3         # ความเร็วสำรองขั้นต่ำ (ส่วนของ speed_limit)
OCCUPANCY_FREE_THRESH: float = 0.05     # ถ้าบางมาก ให้ใช้ speed_limit เต็ม
SMOOTHING_MIN_SPEED: float = 1.5        # เกณฑ์ความเร็วหลัง smooth

PROGRESS_EVERY_SIMSEC: int = 30         # print ความคืบหน้าทุก X วินาทีจำลอง

# TLS parameters
TLS_LOOKAHEAD_LIMIT: float = 300.0      # สแกนไปข้างหน้าสูงสุด (s) เพื่อหาเขียว
TLS_REFRESH_EVERY: int = 60             # รีเฟรชนิยาม TLS เป็นงวด (s)
TLS_CAP_PER_MOVEMENT: float = 25.0      # เพดาน delay ต่อ movement (กัน overreact)

NO_OPT: bool = False                    # เปิด optimize เส้นทาง
TAG: str = ""                           # ต่อท้ายชื่อไฟล์สรุป

# -------------------------------------------------------------------
# 3) GLOBAL ROLLING BUFFERS (สำหรับ smooth ค่าความเร็ว/การครอบครอง)
# -------------------------------------------------------------------
WINDOW_N: int = 10
EDGE_BUF: Dict[str, Dict[str, deque]] = {}  # edge_id -> {"speed": deque, "occ": deque}


# -------------------------------------------------------------------
# 4) CONFIG HELPERS — หาเส้นทางไฟล์จาก .sumocfg
# -------------------------------------------------------------------
def resolve_cfg_path() -> str:
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if os.path.isabs(arg) and os.path.exists(arg):
            return arg
        cand = os.path.abspath(arg)
        if os.path.exists(cand):
            return cand

    env_cfg = os.environ.get("SUMO_CFG")
    if env_cfg:
        cand = os.path.abspath(env_cfg)
        if os.path.exists(cand):
            return cand

    if os.path.exists("osm.sumocfg"):
        return os.path.abspath("osm.sumocfg")

    all_cfgs = sorted(glob.glob("*.sumocfg"))
    if all_cfgs:
        return os.path.abspath(all_cfgs[0])

    raise FileNotFoundError(
        "No .sumocfg found. Pass it as argv[1] or set SUMO_CFG or place osm.sumocfg in CWD."
    )


def read_net_from_cfg(cfg_path: str) -> Optional[str]:
    try:
        root = ET.parse(cfg_path).getroot()
        tag = root.find(".//input/net-file")
        if tag is not None and "value" in tag.attrib:
            return os.path.abspath(os.path.join(os.path.dirname(cfg_path), tag.attrib["value"]))
    except ET.ParseError:
        pass

    nets = glob.glob("*.net.xml*")  # เผื่อ .gz
    return os.path.abspath(nets[0]) if nets else None


def read_route_files_from_cfg(cfg_path: str) -> List[str]:
    try:
        root = ET.parse(cfg_path).getroot()
        tag = root.find(".//input/route-files")
        if tag is None or "value" not in tag.attrib:
            return []
        base = os.path.dirname(cfg_path)
        return [os.path.abspath(os.path.join(base, v)) for v in tag.attrib["value"].split()]
    except ET.ParseError:
        return []


# -------------------------------------------------------------------
# 5) LANE/EDGE UTILITIES — ฟังก์ชันช่วยเกี่ยวกับเลน/เอดจ์
# -------------------------------------------------------------------
def _safe_iter_out_lanes(lane_id: str) -> List[str]:
    try:
        links = traci.lane.getLinks(lane_id)
    except traci.TraCIException:
        return []

    outs: List[str] = []
    for lk in links:
        try:
            to_lane = lk[0] if isinstance(lk, (list, tuple)) else getattr(lk, "toLane", None)
        except Exception:
            to_lane = None
        if to_lane:
            outs.append(to_lane)
    return outs


def _lane_allows_class(lane_id: str, vehicle_class: Optional[str]) -> bool:
    if not vehicle_class:
        return True
    try:
        allowed = set(traci.lane.getAllowed(lane_id))
        disallowed = set(traci.lane.getDisallowed(lane_id))
        if vehicle_class in disallowed:
            return False
        if allowed and (vehicle_class not in allowed):
            return False
        return True
    except traci.TraCIException:
        return True


def get_vehicle_class(veh_id: str) -> Optional[str]:
    try:
        vtype = traci.vehicle.getTypeID(veh_id)
        return traci.vehicletype.getVehicleClass(vtype)
    except traci.TraCIException:
        return None


def next_edges_allowed_from_current_lane(veh_id: str) -> Set[str]:
    try:
        lane_id = traci.vehicle.getLaneID(veh_id)
        return {ln.split("_")[0] for ln in _safe_iter_out_lanes(lane_id)}
    except traci.TraCIException:
        return set()


def lane_has_link_to_edge(lane_id: str, edge_id: str) -> bool:
    for toLane in _safe_iter_out_lanes(lane_id):
        if toLane.split("_")[0] == edge_id:
            return True
    return False


def is_uturn_pair(a: str, b: str) -> bool:
    return (a == "-" + b) or (b == "-" + a)


def has_edge_connection_any_lane(a: str, b: str) -> bool:
    try:
        for lane_id in traci.edge.getLaneIDs(a):
            if lane_has_link_to_edge(lane_id, b):
                return True
    except Exception:
        pass
    return False


# -------------------------------------------------------------------
# 6) BUILD EDGE GRAPH — สร้างกราฟ (nodes=edges) จาก TraCI
# -------------------------------------------------------------------
def _lanes_of_edge(eid: str) -> list[str]:
    """หา lane ทั้งหมดของ edge eid โดยใช้ prefix (เพราะ TraCI ไม่มี edge.getLaneIDs)"""
    lanes = []
    try:
        for lid in traci.lane.getIDList():
            if lid.startswith(eid + "_"):
                lanes.append(lid)
    except Exception:
        pass
    return lanes


def build_edge_graph_from_traci(vehicle_class: Optional[str] = None) -> nx.DiGraph:
    """
    สร้างกราฟ (nodes = edges):
      - ใส่โหนดครบทุก edge (non-internal)
      - length/speed_limit จาก lane ทั้งหมดของ edge
      - การเชื่อมต่อกรองตาม vehicle_class ที่เลนอนุญาตจริง
    """
    G = nx.DiGraph()

    # ---------- 1) โหนด ----------
    for eid in traci.edge.getIDList():
        if (not eid) or eid.startswith(":"):
            continue

        lane_ids = _lanes_of_edge(eid)

        lengths, speeds = [], []
        for lid in lane_ids:
            try:
                lengths.append(traci.lane.getLength(lid))
            except Exception:
                pass
            try:
                speeds.append(traci.lane.getMaxSpeed(lid))
            except Exception:
                pass

        length = max(lengths) if lengths else 1.0
        speed_limit = min(speeds) if speeds else 13.9
        G.add_node(eid, length=length, speed_limit=speed_limit)

    # ---------- 2) เส้นเชื่อม ----------
    for lane_id in traci.lane.getIDList():
        base_edge = lane_id.split("_")[0] if "_" in lane_id else lane_id
        if (not base_edge) or base_edge.startswith(":") or (base_edge not in G):
            continue
        if not _lane_allows_class(lane_id, vehicle_class):
            continue

        for toLane in _safe_iter_out_lanes(lane_id):
            to_edge = toLane.split("_")[0] if "_" in toLane else toLane
            if (not to_edge) or to_edge.startswith(":") or (to_edge not in G):
                continue
            if not _lane_allows_class(toLane, vehicle_class):
                continue

            G.add_edge(base_edge, to_edge)

    return G


# -------------------------------------------------------------------
# 7) COST MODEL — expected_speed (base weights only; NO global TLS)
# -------------------------------------------------------------------
def expected_speed(out_edge: str, speed_limit: float) -> Tuple[float, float]:
    try:
        occ = traci.edge.getLastStepOccupancy(out_edge)
    except Exception:
        occ = 0.0
    try:
        mean_speed = traci.edge.getLastStepMeanSpeed(out_edge)
    except Exception:
        mean_speed = None

    # validate
    if not isinstance(occ, (int, float)) or occ != occ:
        occ = 0.0
    occ = max(0.0, min(1.0, float(occ)))

    if not isinstance(mean_speed, (int, float)) or mean_speed <= 0:
        mean_speed = speed_limit * MIN_SPEED_FRACTION

    # --- no buffer / no averaging ---
    if occ < OCCUPANCY_FREE_THRESH:
        use_speed = max(speed_limit, 0.1)
    elif mean_speed > SMOOTHING_MIN_SPEED:
        use_speed = mean_speed
    else:
        use_speed = max(speed_limit * MIN_SPEED_FRACTION, 0.1)

    return use_speed, occ


def compute_edge_weights_for_class(veh_class_key: str, G: nx.DiGraph) -> None:
    # ใส่เฉพาะ base_tt * dens_mult เป็น "w_base" และ sync ไป "w"
    node_stat: Dict[str, Tuple[float, float]] = {}
    for v in G.nodes:
        nd = G.nodes[v]
        speed_limit = nd.get("speed_limit", 13.9)
        use_speed, occ = expected_speed(v, speed_limit)
        base_tt = nd.get("length", 1.0) / max(use_speed, 0.1)
        dens_mult = 1.0 + DENSITY_ALPHA * float(occ)
        node_stat[v] = (base_tt, dens_mult)

    for u, v in G.edges:
        base_tt, dens_mult = node_stat[v]
        w_base = base_tt * dens_mult
        G.edges[u, v]["w_base"] = w_base
        G.edges[u, v]["w"] = w_base  # ค่าเริ่มต้นเท่ากัน (จะ override รายคันเฉพาะขอบฟ้า)


# -------------------------------------------------------------------
# 8) TLS ETA-aware utilities
# -------------------------------------------------------------------
def cache_tls_definitions() -> Dict[str, Dict]:
    tls_defs: Dict[str, Dict] = {}
    for tls_id in traci.trafficlight.getIDList():
        progs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        if not progs:
            continue
        phases = [(p.state, float(p.duration)) for p in progs[0].phases]
        tls_defs[tls_id] = {"phases": phases, "cycle": sum(d for _, d in phases)}
    return tls_defs


def build_tls_linkmap() -> Dict[str, Dict[Tuple[str, str], int]]:
    linkmap: Dict[str, Dict[Tuple[str, str], int]] = {}
    for tls_id in traci.trafficlight.getIDList():
        mapping: Dict[Tuple[str, str], int] = {}
        for sig_idx, links in enumerate(traci.trafficlight.getControlledLinks(tls_id)):
            for triple in links:
                try:
                    in_lane, out_lane, _ = triple
                except Exception:
                    try:
                        in_lane, out_lane = triple[0], triple[1]
                    except Exception:
                        continue
                mapping[(in_lane.split("_")[0], out_lane.split("_")[0])] = sig_idx
        linkmap[tls_id] = mapping
    return linkmap


def _is_green(state: str, idx: int) -> bool:
    return idx < len(state) and state[idx] in ("G", "g")


def tls_delay_seconds_eta(tls_id: str, sig_index: int, t_arrival: float, tls_defs: Dict[str, Dict]) -> float:
    """
    ETA-aware delay: เวลาที่ต้องรอจนถึงเขียว ณ 't_arrival' (เวลาที่รถไปถึง movement นั้นจริง)
    """
    defs = tls_defs.get(tls_id)
    if not defs:
        return 0.0
    phases = defs.get("phases") or []
    cycle = float(defs.get("cycle", 0.0))
    if cycle <= 0.0 or not phases:
        return 0.0

    try:
        cur_phase_idx = traci.trafficlight.getPhase(tls_id)
        next_switch = float(traci.trafficlight.getNextSwitch(tls_id))
        now = float(traci.simulation.getTime())
        remain_now = max(0.0, next_switch - now)   # time left in current phase at 'now'
    except Exception:
        return 0.0

    n = len(phases)
    durs = [float(d) for _, d in phases]

    # เดินเวลาไปข้างหน้าจาก now -> t_arrival
    dt = max(0.0, t_arrival - now)
    idx = cur_phase_idx
    rem = remain_now
    tleft = dt
    while tleft > rem:
        tleft -= rem
        idx = (idx + 1) % n
        rem = durs[idx]

    # ตอนถึง: เราอยู่ในเฟส idx (เหลือ rem - tleft จะจบเฟส)
    state_at_arr = phases[idx][0]
    if _is_green(state_at_arr, sig_index):
        return 0.0

    # หาเวลาจนถึงเขียวครั้งถัดไปจากจุดนี้
    delay = rem - tleft
    scanned = 0.0
    scan_idx = (idx + 1) % n
    lookahead = float(globals().get("TLS_LOOKAHEAD_LIMIT", cycle))
    while scanned < lookahead:
        st, dur = phases[scan_idx]
        if _is_green(st, sig_index):
            return max(0.0, delay)
        delay += float(dur)
        scanned += float(dur)
        scan_idx = (scan_idx + 1) % n

    return 0.0


def expected_tls_delay_for_movement_eta(in_edge: str, out_edge: str,
                                        t_arrival: float,
                                        tls_defs: Dict[str, Dict],
                                        tls_linkmap: Dict[str, Dict[Tuple[str, str], int]]) -> float:
    for tls_id, mp in tls_linkmap.items():
        idx = mp.get((in_edge, out_edge))
        if idx is not None:
            return tls_delay_seconds_eta(tls_id, idx, t_arrival, tls_defs)
    return 0.0


# -------------------------------------------------------------------
# 9) PREDICTION HELPER (METHOD A) — เวลาที่เหลือจากจุดปัจจุบัน
# -------------------------------------------------------------------
def predict_remaining_time_for_route(
    G: nx.DiGraph,
    current_edge: str,
    lane_id: Optional[str],
    lane_pos: float,
    route_edges: List[str],
) -> float:
    try:
        lane_len = traci.lane.getLength(lane_id) if lane_id is not None else None
    except Exception:
        lane_len = None

    rem_time_cur = 0.0
    if lane_len is not None:
        rem_len = max(0.0, lane_len - lane_pos)
        nd = G.nodes.get(current_edge, {})
        speed_limit = nd.get("speed_limit", 13.9)
        use_speed, _ = expected_speed(current_edge, speed_limit)
        rem_time_cur = rem_len / max(use_speed, 0.1)

    path_time = 0.0
    if len(route_edges) >= 2:
        for u, v in zip(route_edges[:-1], route_edges[1:]):
            # ใช้ w ถ้ามี ไม่งั้นคำนวณคร่าว ๆ
            if (u, v) in G.edges:
                w = G.edges[u, v].get("w", None)
            else:
                w = None
            if w is None:
                nd = G.nodes.get(v, {})
                sl = nd.get("speed_limit", 13.9)
                ln = nd.get("length", 1.0)
                w = (ln / max(sl, 0.1))
            path_time += float(w)

    return rem_time_cur + path_time


# -------------------------------------------------------------------
# 10) MAIN LOOP — รันซิม, reroute, เก็บสถิติ, สรุปผล
# -------------------------------------------------------------------
def main() -> None:
    wall_start = time.perf_counter()

    cfg = resolve_cfg_path()
    net_path = read_net_from_cfg(cfg)
    if not net_path or not os.path.exists(net_path):
        raise FileNotFoundError(f"Cannot locate .net.xml referenced by cfg: {cfg}")

    route_files = read_route_files_from_cfg(cfg)
    for rf in route_files:
        if not os.path.exists(rf):
            raise FileNotFoundError(f"route-file not found: {rf}")

    print(f"[CFG] Using cfg: {cfg}")
    print(f"[NET] Using net: {net_path}")
    if route_files:
        print("[ROUTES] " + ", ".join(route_files))

    agg_csv = f"aggregates{TAG}.csv"

    sumo_bin = sumolib.checkBinary("sumo")
    cmd = [sumo_bin, "-c", cfg, "--quit-on-end"]
    print("[INFO] Launching:", " ".join(cmd))
    traci.start(cmd)

    # TLS caches
    tls_defs = cache_tls_definitions()
    tls_linkmap = build_tls_linkmap()
    last_tls_refresh_bucket = -1

    # ---- ตัวแปรสะสมสำหรับ Method A
    maeV_abs_sum: float = 0.0
    maeV_count: int = 0
    PER_VEHICLE_MIN_HORIZON: float = 0.0

    try:
        last_reroute = defaultdict(lambda: -1e9)

        # สถานะรายคัน
        vehicle_states: Dict[str, Dict[str, float]] = {}

        # ตัวสะสมภาพรวม
        total_travel_time: float = 0.0
        total_waiting_time: float = 0.0
        total_co2_mg: float = 0.0
        total_fuel_ml: float = 0.0
        vehicle_count: int = 0

        # แคชกราฟแยกตาม class
        edge_graph_cache: Dict[str, nx.DiGraph] = {}
        last_progress_bucket: int = -1

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            # step size (s)
            try:
                step_sec = traci.simulation.getDeltaT() / 1000.0
            except Exception:
                step_sec = 1.0

            # TLS light refresh
            tls_bucket = int(sim_time) // TLS_REFRESH_EVERY
            if tls_bucket != last_tls_refresh_bucket:
                last_tls_refresh_bucket = tls_bucket
                try:
                    tls_defs = cache_tls_definitions()
                    tls_linkmap = build_tls_linkmap()
                except Exception:
                    pass

            # (1) อัปเดตค่าสะสมรายคันที่ยังวิ่งอยู่
            active_ids: Set[str] = set(traci.vehicle.getIDList())
            for vid, vs in list(vehicle_states.items()):
                if vid not in active_ids:
                    continue
                try:
                    curr_acc_wait = traci.vehicle.getAccumulatedWaitingTime(vid)
                    delta_wait = max(0.0, curr_acc_wait - vs.get("last_wait_acc", 0.0))
                    vs["waiting_time"] += delta_wait
                    vs["last_wait_acc"] = curr_acc_wait
                    vs["co2_mg"] += traci.vehicle.getCO2Emission(vid) * step_sec
                    vs["fuel_ml"] += traci.vehicle.getFuelConsumption(vid) * step_sec
                except traci.TraCIException:
                    pass

            # (2) ลงทะเบียนคันที่เพิ่งออกตัว
            for vid in active_ids:
                if vid in vehicle_states:
                    continue
                try:
                    vehicle_states[vid] = {
                        "depart_time": sim_time,
                        "waiting_time": 0.0,
                        "last_wait_acc": traci.vehicle.getAccumulatedWaitingTime(vid),
                        "co2_mg": 0.0,
                        "fuel_ml": 0.0,
                    }
                except traci.TraCIException:
                    vehicle_states[vid] = {
                        "depart_time": sim_time,
                        "waiting_time": 0.0,
                        "last_wait_acc": 0.0,
                        "co2_mg": 0.0,
                        "fuel_ml": 0.0,
                    }

            # (3) คำนวณน้ำหนัก base + reroute
            if not NO_OPT:

                def compute_edge_weights_for_this_step(classes_in_step: Set[str]) -> None:
                    for cls in classes_in_step:
                        if cls not in edge_graph_cache:
                            real_cls = None if cls == "_ANY_" else cls
                            edge_graph_cache[cls] = build_edge_graph_from_traci(real_cls)
                        compute_edge_weights_for_class(cls, edge_graph_cache[cls])

                step_classes: Set[str] = {get_vehicle_class(vid) or "_ANY_" for vid in active_ids}
                compute_edge_weights_for_this_step(step_classes)

                # reroute รายคัน
                for vid in active_ids:
                    if (sim_time - last_reroute[vid]) < REROUTE_PERIOD:
                        continue

                    try:
                        lane_id = traci.vehicle.getLaneID(vid)
                        lane_pos = traci.vehicle.getLanePosition(vid)
                        lane_len = traci.lane.getLength(lane_id)
                        if (lane_len - lane_pos) < NEAR_JUNCTION_SKIP_DIST:
                            continue
                    except traci.TraCIException:
                        pass

                    cur_edge = traci.vehicle.getRoadID(vid)
                    if (not cur_edge) or cur_edge.startswith(":"):
                        continue

                    try:
                        route = traci.vehicle.getRoute(vid)
                    except traci.TraCIException:
                        continue
                    if not route:
                        continue

                    dest_edge = route[-1]
                    if (not dest_edge) or dest_edge.startswith(":"):
                        continue

                    veh_class = get_vehicle_class(vid)
                    key = veh_class or "_ANY_"
                    G = edge_graph_cache[key]

                    if cur_edge not in G or dest_edge not in G:
                        try:
                            traci.vehicle.rerouteTraveltime(vid)
                            last_reroute[vid] = sim_time
                            print(f"[WARN] Vehicle {vid}: graph missing for class '{key}' or edges")
                        except traci.TraCIException:
                            pass
                        continue

                    # ===== ETA-aware TLS: ปรับเฉพาะ successors ของขอบฟ้าปัจจุบัน =====
                    try:
                        # เวลาไปถึงหัวแยก (ปลายขอบของ cur_edge) ของคันนี้
                        now = float(traci.simulation.getTime())
                        nd_cur = G.nodes.get(cur_edge, {})
                        sl_cur = nd_cur.get("speed_limit", 13.9)

                        try:
                            lane_id = traci.vehicle.getLaneID(vid)
                            lane_pos = traci.vehicle.getLanePosition(vid)
                            lane_len = traci.lane.getLength(lane_id)
                        except traci.TraCIException:
                            lane_id, lane_pos, lane_len = None, 0.0, None

                        if lane_len is not None:
                            use_speed_cur, _ = expected_speed(cur_edge, sl_cur)
                            time_to_stopline = max(0.0, (lane_len - lane_pos) / max(use_speed_cur, 0.1))
                        else:
                            time_to_stopline = 0.0

                        t_arrival_at_tls = now + time_to_stopline

                        # เตรียมคืนค่า (restore) หลังคำนวณเส้นทาง
                        touched: List[Tuple[str, str, float]] = []
                        for v in G.successors(cur_edge):
                            w_base = G.edges[cur_edge, v].get("w_base", None)
                            if w_base is None:
                                # sync หากยังไม่มี
                                w_base = G.edges[cur_edge, v].get("w", 0.0)
                                G.edges[cur_edge, v]["w_base"] = w_base
                            tls_del = expected_tls_delay_for_movement_eta(
                                cur_edge, v, t_arrival_at_tls, tls_defs, tls_linkmap
                            )
                            # เพดานเพื่อกัน overreact
                            tls_del = min(max(0.0, tls_del), TLS_CAP_PER_MOVEMENT)

                            old_w = G.edges[cur_edge, v].get("w", w_base)
                            new_w = float(w_base) + float(tls_del)
                            if abs(new_w - old_w) > 1e-9:
                                touched.append((cur_edge, v, old_w))
                                G.edges[cur_edge, v]["w"] = new_w

                        try:
                            edge_path_new = nx.shortest_path(G, cur_edge, dest_edge, weight="w")
                        except Exception:
                            # restore ก่อน fallback
                            for u, v, old_w in touched:
                                G.edges[u, v]["w"] = old_w
                            try:
                                traci.vehicle.rerouteTraveltime(vid)
                                last_reroute[vid] = sim_time
                            except traci.TraCIException:
                                pass
                            continue
                        finally:
                            # คืนค่า w เดิมหลังคำนวณเส้นทาง (กันสะสมข้ามคัน)
                            for u, v, old_w in touched:
                                G.edges[u, v]["w"] = old_w

                    except Exception as e:
                        # ถ้า TLS มีปัญหา ให้ fallback
                        try:
                            edge_path_new = nx.shortest_path(G, cur_edge, dest_edge, weight="w")
                        except Exception:
                            try:
                                traci.vehicle.rerouteTraveltime(vid)
                                last_reroute[vid] = sim_time
                            except traci.TraCIException:
                                pass
                            continue

                    new_route = list(edge_path_new)

                    allowed_from_lane = next_edges_allowed_from_current_lane(vid)
                    if len(new_route) >= 2 and allowed_from_lane and (new_route[1] not in allowed_from_lane):
                        print(f"[WARN] Vehicle {vid}: path not allowed from current lane")
                        continue
                    if len(new_route) >= 2 and is_uturn_pair(new_route[0], new_route[1]):
                        print(f"[WARN] Vehicle {vid}: path has U-turn at start")
                        continue

                    try:
                        traci.vehicle.setRoute(vid, new_route)
                        print(f"[INFO] Vehicle {vid}: rerouted via {len(new_route)} edges")
                        last_reroute[vid] = sim_time

                        vs = vehicle_states.get(vid)
                        if vs is not None:
                            try:
                                lane_id = traci.vehicle.getLaneID(vid)
                                lane_pos = traci.vehicle.getLanePosition(vid)
                            except traci.TraCIException:
                                lane_id, lane_pos = None, 0.0

                            pred_rem = predict_remaining_time_for_route(G, cur_edge, lane_id, lane_pos, new_route)
                            vs["last_decision_time"] = float(sim_time)
                            vs["last_pred_remaining"] = float(pred_rem)

                    except traci.TraCIException:
                        continue

                # เติม snapshot เริ่มต้นให้คันที่ยังไม่เคย snapshot
                for vid in active_ids:
                    vs = vehicle_states.get(vid)
                    if (not vs) or ("last_decision_time" in vs):
                        continue
                    try:
                        cur_edge = traci.vehicle.getRoadID(vid)
                        if (not cur_edge) or cur_edge.startswith(":"):
                            continue
                        route = traci.vehicle.getRoute(vid)
                        if not route:
                            continue

                        veh_class = get_vehicle_class(vid)
                        key = veh_class or "_ANY_"
                        G = edge_graph_cache.get(key)
                        if (not G) or (cur_edge not in G) or (route[-1] not in G):
                            continue

                        lane_id = traci.vehicle.getLaneID(vid)
                        lane_pos = traci.vehicle.getLanePosition(vid)
                        pred_rem = predict_remaining_time_for_route(G, cur_edge, lane_id, lane_pos, route)
                        vs["last_decision_time"] = float(sim_time)
                        vs["last_pred_remaining"] = float(pred_rem)
                    except traci.TraCIException:
                        pass

            # (4) เมื่อรถถึงปลายทาง -> อัปเดตสถิติ + MAE
            for vid in traci.simulation.getArrivedIDList():
                vs = vehicle_states.get(vid)
                if vs is None:
                    continue

                try:
                    travel_time = sim_time - vs.get("depart_time", sim_time)
                    total_travel_time += travel_time
                    total_waiting_time += vs.get("waiting_time", 0.0)
                    total_co2_mg += vs.get("co2_mg", 0.0)
                    total_fuel_ml += vs.get("fuel_ml", 0.0)
                    vehicle_count += 1
                except Exception:
                    pass

                try:
                    last_dec_t = vs.get("last_decision_time", None)
                    pred_rem = vs.get("last_pred_remaining", None)
                    if (last_dec_t is not None) and (pred_rem is not None):
                        actual_rem = max(0.0, float(sim_time) - float(last_dec_t))
                        if actual_rem >= PER_VEHICLE_MIN_HORIZON:
                            abs_err = abs(actual_rem - float(pred_rem))
                            maeV_abs_sum += abs_err
                            maeV_count += 1
                except Exception:
                    pass
                finally:
                    vehicle_states.pop(vid, None)

            # (5) แสดงความคืบหน้าเป็นงวดๆ
            bucket = int(sim_time) // PROGRESS_EVERY_SIMSEC
            if bucket != last_progress_bucket:
                last_progress_bucket = bucket
                print(f"{vehicle_count}", flush=True)

        # ======= สรุปผล =======
        avg_travel = (total_travel_time / vehicle_count) if vehicle_count else 0.0
        avg_wait = (total_waiting_time / vehicle_count) if vehicle_count else 0.0
        avg_co2_mg = (total_co2_mg / vehicle_count) if vehicle_count else 0.0
        avg_fuel_ml = (total_fuel_ml / vehicle_count) if vehicle_count else 0.0

        if maeV_count > 0:
            maeV = maeV_abs_sum / maeV_count
        else:
            maeV = 0.0
        rmae_global = (maeV / avg_travel) if (vehicle_count > 0 and avg_travel > 1e-12) else 0.0

        write_header = not os.path.exists(agg_csv)
        with open(agg_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if write_header:
                w.writerow([
                    "vehicles", "avg_travel_time_s", "avg_waiting_time_s",
                    "avg_CO2_mg", "avg_fuel_ml",
                    "MAE_per_vehicle_s", "RMAE_global"
                ])
            w.writerow([
                vehicle_count, f"{avg_travel:.3f}", f"{avg_wait:.3f}",
                f"{avg_co2_mg:.3f}", f"{avg_fuel_ml:.3f}",
                f"{maeV:.3f}", f"{rmae_global:.6f}"
            ])

        wall_end = time.perf_counter()
        processing_time = wall_end - wall_start

        print("\n===== Simulation Summary =====")
        print(f"Vehicles simulated (included):  {vehicle_count}")
        print(f"Average travel time:            {avg_travel:.3f} s")
        print(f"Average waiting time:           {avg_wait:.3f} s")
        print(f"Average CO2 emission:           {avg_co2_mg:.3f} mg/vehicle")
        print(f"Average fuel consumption:       {avg_fuel_ml:.3f} ml/vehicle")
        print(f"Total CO2 emission:             {total_co2_mg:.3f} mg")
        print(f"Total fuel consumption:         {total_fuel_ml:.3f} ml")
        print(f"(Aggregates written to: {agg_csv})")

        print("\n--- Forecast accuracy (Method A: per vehicle, last snapshot) ---")
        print(f"MAE per vehicle:                {maeV:.3f} s")
        print(f"RMAE (global = MAE/avg_tt):     {rmae_global:.6f}")

        print(f"\nProcessing time (wall-clock):   {processing_time:.2f} s")

    finally:
        try:
            traci.close(False)
        except Exception:
            pass


# Entry point
if __name__ == "__main__":
    main()
