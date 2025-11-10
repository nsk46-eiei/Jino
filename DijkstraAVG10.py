# FinalnoTLS.py
# -*- coding: utf-8 -*-

"""
========================================
SUMO Routing (No TLS) — Readable Layout
========================================

สารบัญ
  1) Imports & Bootstrap
  2) Parameters (ค่าตั้งค่า)
  3) Global Rolling Buffers
  4) Config Helpers           -> หาไฟล์ .sumocfg / .net.xml / route-files
  5) Lane/Edge Utilities      -> ฟังก์ชันช่วยเกี่ยวกับเลน/เอดจ์
  6) Build Edge Graph         -> สร้างกราฟ (nodes=edges) จาก TraCI
  7) Cost Model (No TLS)      -> expected_speed + คำนวณน้ำหนัก w
  8) Prediction Helper        -> ประเมินเวลาที่เหลือบนเส้นทางปัจจุบัน (Method A)
  9) Main Loop                -> รันซิมูเลชัน, reroute, เก็บสถิติ, สรุปผล
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
REROUTE_PERIOD: float = 1.0           # คาบเวลาการ reroute ต่อคัน (วินาทีจำลอง)
NEAR_JUNCTION_SKIP_DIST: float = 20.0   # ใกล้จุดตัดให้ข้ามการ setRoute
DENSITY_ALPHA: float = 2             # ค่าถ่วงความหนาแน่น
MIN_SPEED_FRACTION: float = 0.3        # ความเร็วสำรองขั้นต่ำ (ส่วนของ speed_limit)
OCCUPANCY_FREE_THRESH: float = 0.05     # ถ้าบางมาก ให้ใช้ speed_limit เต็ม
SMOOTHING_MIN_SPEED: float = 1.5       # เกณฑ์ความเร็วหลัง smooth

PROGRESS_EVERY_SIMSEC: int = 30        # print ความคืบหน้าทุก X วินาทีจำลอง

NO_OPT: bool = False                   # เปิด optimize เส้นทาง (เดิม True)
TAG: str = ""                          # ต่อท้ายชื่อไฟล์สรุป

# -------------------------------------------------------------------
# 3) GLOBAL ROLLING BUFFERS (สำหรับ smooth ค่าความเร็ว/การครอบครอง)
# -------------------------------------------------------------------
WINDOW_N: int = 10
EDGE_BUF: Dict[str, Dict[str, deque]] = {}  # edge_id -> {"speed": deque, "occ": deque}


# -------------------------------------------------------------------
# 4) CONFIG HELPERS — หาเส้นทางไฟล์จาก .sumocfg
# -------------------------------------------------------------------
def resolve_cfg_path() -> str:
    """
    ลำดับการหา .sumocfg:
      1) sys.argv[1]
      2) env: SUMO_CFG
      3) ./osm.sumocfg
      4) ไฟล์ *.sumocfg อันแรกในโฟลเดอร์
    """
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
    """คืน path ของ .net.xml ที่อ้างใน .sumocfg (ถ้า parse ไม่ได้ จะลองหา *.net.xml ในโฟลเดอร์)"""
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
    """คืนลิสต์ route-files (absolute paths) จาก .sumocfg"""
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
    """คืนลิสต์เลนปลายทางที่เชื่อมจาก lane_id (ถ้า error คืนลิสต์ว่าง)"""
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
    """เลนนี้อนุญาต vehicle_class หรือไม่ (None = ผ่าน)"""
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
    """อ่าน vehicle class จาก type ของรถ"""
    try:
        vtype = traci.vehicle.getTypeID(veh_id)
        return traci.vehicletype.getVehicleClass(vtype)
    except traci.TraCIException:
        return None


def next_edges_allowed_from_current_lane(veh_id: str) -> Set[str]:
    """คืนชุด edge ที่ไปต่อได้จากเลนปัจจุบันของรถ (ฐานเป็น edge_id)"""
    try:
        lane_id = traci.vehicle.getLaneID(veh_id)
        return {ln.split("_")[0] for ln in _safe_iter_out_lanes(lane_id)}
    except traci.TraCIException:
        return set()


def lane_has_link_to_edge(lane_id: str, edge_id: str) -> bool:
    """เลนนี้มีลิงก์ไปยังเลนที่อยู่บน edge_id หรือไม่"""
    for toLane in _safe_iter_out_lanes(lane_id):
        if toLane.split("_")[0] == edge_id:
            return True
    return False


def is_uturn_pair(a: str, b: str) -> bool:
    """ตรวจ u-turn รูปแบบ 'x' <-> '-x'"""
    return (a == "-" + b) or (b == "-" + a)


def has_edge_connection_any_lane(a: str, b: str) -> bool:
    """edge a เชื่อมไป edge b หรือไม่ (เช็คทุกเลนของ a)"""
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
            # lane ปกติจะเป็นรูป edgeId_index เช่น E123_0, E123_1
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

        # ใช้ตัวช่วยด้านบนแทน traci.edge.getLaneIDs(eid)
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
# 7) COST MODEL (No TLS) — expected_speed & การคำนวณน้ำหนัก w
# -------------------------------------------------------------------
def expected_speed(out_edge: str, speed_limit: float) -> Tuple[float, float]:
    """
    ใช้ค่าเฉลี่ยแบบกลิ้ง (rolling average) 10 สเต็ป
    สำหรับทั้ง mean speed และ occupancy แล้วค่อยตัดสินใจ use_speed
    """
    # --- อ่าน occupancy ล่าสุด ---
    try:
        occ = traci.edge.getLastStepOccupancy(out_edge)
    except Exception:
        occ = 0.0

    # --- อ่าน mean speed ล่าสุด ---
    try:
        mean_speed = traci.edge.getLastStepMeanSpeed(out_edge)
    except Exception:
        mean_speed = None

    # --- validate mean speed ---
    if mean_speed <= 0:
        print(f"[WARN] Edge {out_edge}: mean_speed={mean_speed} invalid, using fallback")
        mean_speed = speed_limit * MIN_SPEED_FRACTION

    # --- อัปเดต rolling buffers ---
    buf = EDGE_BUF.setdefault(out_edge, {"speed": deque(maxlen=WINDOW_N),
                                         "occ": deque(maxlen=WINDOW_N)})
    buf["speed"].append(float(mean_speed))
    buf["occ"].append(float(occ))

    # --- คำนวณค่าเฉลี่ย 10 สเต็ป (หรือเท่าที่มี) ---
    sm_speed = (sum(buf["speed"]) / len(buf["speed"])) if buf["speed"] else float(mean_speed)
    sm_occ = (sum(buf["occ"]) / len(buf["occ"])) if buf["occ"] else float(occ)

    # --- ตัดสินใจจากค่าเฉลี่ย ---
    if sm_occ < OCCUPANCY_FREE_THRESH:
        # โล่งมาก → ไว้ใจ speed_limit
        use_speed = max(speed_limit, 0.1)
    elif sm_speed > SMOOTHING_MIN_SPEED:
        # มีรถแต่ยังเคลื่อนเร็ว → ใช้ความเร็วเฉลี่ยจริง
        use_speed = sm_speed
    else:
        # หนาแน่นและช้า → ใช้ fallback (ส่วนของ speed limit)
        use_speed = max(speed_limit * MIN_SPEED_FRACTION, 0.1)

    return use_speed, sm_occ


# -------------------------------------------------------------------
# 8) PREDICTION HELPER (METHOD A) — เวลาที่เหลือจากจุดปัจจุบัน
# -------------------------------------------------------------------
def predict_remaining_time_for_route(
    G: nx.DiGraph,
    current_edge: str,
    lane_id: Optional[str],
    lane_pos: float,
    route_edges: List[str],
) -> float:
    """
    ประเมินเวลาที่เหลือ:
      (1) ส่วนที่เหลือบนเลนปัจจุบันของ current_edge  -> ใช้ expected_speed(current_edge)
      (2) บวกน้ำหนัก 'w' ของขอบถัดๆ ไปใน route_edges
          - ถ้าไม่มี w ให้ประมาณ ln/speed_limit ของ node v
    """
    # เศษระยะบนเลนปัจจุบัน
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

    # เวลาสำหรับขอบถัดไป
    path_time = 0.0
    if len(route_edges) >= 2:
        for u, v in zip(route_edges[:-1], route_edges[1:]):
            w = G.edges[u, v].get("w", None) if (u, v) in G.edges else None
            if w is None:
                nd = G.nodes.get(v, {})
                sl = nd.get("speed_limit", 13.9)
                ln = nd.get("length", 1.0)
                w = (ln / max(sl, 0.1))
            path_time += float(w)

    return rem_time_cur + path_time


# -------------------------------------------------------------------
# 9) MAIN LOOP — รันซิม, reroute, เก็บสถิติ, สรุปผล
# -------------------------------------------------------------------
def main() -> None:
    wall_start = time.perf_counter()

    # -- เตรียมไฟล์จำเป็น
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

    # -- เริ่ม SUMO (เปลี่ยนเป็น 'sumo-gui' ได้ถ้าต้องการ GUI)
    sumo_bin = sumolib.checkBinary("sumo")
    cmd = [sumo_bin, "-c", cfg, "--quit-on-end"]
    print("[INFO] Launching:", " ".join(cmd))
    traci.start(cmd)

    # ---- ตัวแปรสะสมสำหรับ Method A (per-vehicle last snapshot)
    maeV_abs_sum: float = 0.0
    maeV_count: int = 0
    PER_VEHICLE_MIN_HORIZON: float = 0.0  # ตั้ง >0 เพื่อกรอง snapshot ใกล้ถึงมากๆ

    try:
        last_reroute = defaultdict(lambda: -1e9)  # เวลา reroute ล่าสุดของแต่ละคัน

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

        # ======= วนรอบซิมูเลชัน =======
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            # step size (s)
            try:
                step_sec = traci.simulation.getDeltaT() / 1000.0
            except Exception:
                step_sec = 1.0

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
                        # snapshot fields (จะใส่ภายหลังเมื่อ reroute/setRoute)
                    }
                except traci.TraCIException:
                    vehicle_states[vid] = {
                        "depart_time": sim_time,
                        "waiting_time": 0.0,
                        "last_wait_acc": 0.0,
                        "co2_mg": 0.0,
                        "fuel_ml": 0.0,
                    }

            # (3) คำนวณน้ำหนัก + reroute
            if not NO_OPT:

                def compute_edge_weights_for_class(veh_class_key: str, G: nx.DiGraph) -> None:
                    """
                    คิดค่า cost ที่ node v (คือ edge ปลายทาง):
                      base_tt(v) = length(v) / expected_speed(v)
                      cost(v)    = base_tt(v) * (1 + DENSITY_ALPHA * occ(v))
                    แล้วเซ็ต w สำหรับ edge (u,v) = cost(v)
                    """
                    per_out_edge_cost: Dict[str, float] = {}

                    # node v -> คำนวณ cost
                    for v in G.nodes:
                        nd = G.nodes[v]
                        speed_limit = nd.get("speed_limit", 13.9)
                        use_speed, occ = expected_speed(v, speed_limit)
                        base_tt = nd.get("length", 1.0) / max(use_speed, 0.1)
                        per_out_edge_cost[v] = base_tt * (1.0 + DENSITY_ALPHA * float(occ))

                    # กระจายน้ำหนักไปยังแต่ละ (u,v)
                    for u, v in G.edges:
                        G.edges[u, v]["w"] = per_out_edge_cost[v]

                # เตรียมกราฟตาม class ที่มีในสเต็ปนี้
                step_classes: Set[str] = {get_vehicle_class(vid) or "_ANY_" for vid in active_ids}
                for cls in step_classes:
                    if cls not in edge_graph_cache:
                        real_cls = None if cls == "_ANY_" else cls
                        edge_graph_cache[cls] = build_edge_graph_from_traci(real_cls)
                    compute_edge_weights_for_class(cls, edge_graph_cache[cls])

                # reroute รายคัน (เว้นอย่างน้อย REROUTE_PERIOD วินาทีต่อคัน)
                for vid in active_ids:
                    # --- คุมคาบเวลา reroute ต่อคัน ---
                    if (sim_time - last_reroute[vid]) < REROUTE_PERIOD:
                        continue
                    # ----------------------------------

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
                    if key not in edge_graph_cache:
                        edge_graph_cache[key] = build_edge_graph_from_traci(veh_class)

                    G = edge_graph_cache[key]
                    if cur_edge not in G or dest_edge not in G:
                        # กราฟใช้ไม่ได้: fallback เป็น rerouteTraveltime
                        try:
                            traci.vehicle.rerouteTraveltime(vid)
                            last_reroute[vid] = sim_time  # นับเป็นการ reroute ครั้งหนึ่ง
                            print(f"[WARN] Vehicle {vid}: no graph for class '{key}' or missing edges")
                        except traci.TraCIException:
                            pass
                        continue

                    # เส้นทางน้ำหนักต่ำสุด
                    try:
                        edge_path_new = nx.shortest_path(G, cur_edge, dest_edge, weight="w")
                    except Exception:
                        # หา shortest path ไม่ได้: fallback
                        try:
                            traci.vehicle.rerouteTraveltime(vid)
                            last_reroute[vid] = sim_time
                        except traci.TraCIException:
                            pass
                        continue

                    new_route = list(edge_path_new)

                    # เช็กความเป็นไปได้ของขอบถัดไปจากเลนปัจจุบัน + กัน u-turn
                    allowed_from_lane = next_edges_allowed_from_current_lane(vid)
                    if len(new_route) >= 2 and allowed_from_lane and (new_route[1] not in allowed_from_lane):
                        continue
                    if len(new_route) >= 2 and is_uturn_pair(new_route[0], new_route[1]):
                        continue

                    # setRoute + เก็บ snapshot (Method A)
                    try:
                        traci.vehicle.setRoute(vid, new_route)
                        print(f"[INFO] Vehicle {vid}: Rerouted via {len(new_route)}")
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

                # เติม snapshot เริ่มต้นให้คันที่ยังไม่เคย snapshot (ใช้ route ปัจจุบัน)
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

            # (4) เมื่อรถถึงปลายทาง -> อัปเดตสถิติ + คำนวณ MAE (Method A)
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

                # Snapshot สุดท้ายต่อคัน -> ใช้คำนวณ MAE
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
            maeV = maeV_abs_sum / maeV_count           # MAE per vehicle (seconds)
        else:
            maeV = 0.0

        # RMAE (global normalization): MAE / mean(actual travel time ทั้งชุด)
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

        # ---- พิมพ์สรุป
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
