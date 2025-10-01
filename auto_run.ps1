param(
  [string[]] $Cities    = @("CNX1","Singapore1"),
  [string[]] $Densities = @("1","1_5","2","2_5"),
  [string]   $Python    = "",
  [string]   $SumoHome  = "/usr/share/sumo",
  [int]      $TimeoutSecPerRun = 7200,
  [switch]   $Live = $true   # stream output to VS Code terminal by default
)

# -------- Paths & env --------
$Root   = if ($PSScriptRoot) { $PSScriptRoot } else { Split-Path -Parent $MyInvocation.MyCommand.Path }
$LogDir = Join-Path $Root "newmap Logs"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

$env:SUMO_HOME  = $SumoHome
$env:PYTHONPATH = "$($env:SUMO_HOME)/tools:$($env:PYTHONPATH)"
Write-Host "SUMO_HOME = $($env:SUMO_HOME)"
Write-Host "PYTHONPATH= $($env:PYTHONPATH)"

# -------- Helpers --------
function Resolve-Python {
  param([string]$Pref)
  if ($Pref -and (Test-Path $Pref)) { return $Pref }
  $cands = @(
    "C:\Python312\python.exe",
    "C:\Python311\python.exe",
    "C:\Users\$env:USERNAME\AppData\Local\Programs\Python\Python311\python.exe",
    "C:\Program Files\Python311\python.exe",
    "C:\Program Files\Python312\python.exe"
  )
  foreach ($p in $cands) { if (Test-Path $p) { return $p } }
  return "python"  # fallback to PATH
}

$Python = Resolve-Python -Pref $Python
Write-Host "Using Python: $Python"

function Stop-LeftoverSumo {
  $left = Get-Process -Name "sumo","sumo-gui" -ErrorAction SilentlyContinue
  if ($left) {
    Write-Warning ("Killing {0} leftover SUMO process(es)..." -f $left.Count)
    $left | Stop-Process -Force
    Start-Sleep -Seconds 1
  }
}

function Write-LogSummary {
  param([Parameter(Mandatory)][string] $LogPath)
  if (-not (Test-Path $LogPath)) { return }
  $hit = Select-String -Path $LogPath -Pattern '^Simulation ended at time:' | Select-Object -First 1
  if ($null -eq $hit) { return }
  $lines   = Get-Content -LiteralPath $LogPath
  $start   = [int]$hit.LineNumber - 1
  $summary = $lines[$start..($lines.Length - 1)]
  $outPath = [System.IO.Path]::ChangeExtension($LogPath, ".summary.log")
  Set-Content -LiteralPath $outPath -Value $summary
  Write-Host "   -> Wrote summary: $outPath"
}

function Invoke-ProcessWithTimeout {
  param(
    [Parameter(Mandatory)][string] $FilePath,
    [string[]] $ArgumentList = @(),
    [Parameter(Mandatory)][string] $StdOutPath,
    [string] $StdErrPath,
    [int] $TimeoutSec = 7200,
    [string] $WorkingDirectory = $null
  )

  $outDir = Split-Path -Parent $StdOutPath
  if ($outDir) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }

  $useTempErr = $false
  if ([string]::IsNullOrEmpty($StdErrPath) -or ($StdErrPath -eq $StdOutPath)) {
    $StdErrPath = [System.IO.Path]::ChangeExtension($StdOutPath, ".stderr.tmp")
    $useTempErr = $true
  }

  New-Item -ItemType File -Force -Path $StdOutPath | Out-Null
  New-Item -ItemType File -Force -Path $StdErrPath | Out-Null

  # Quote arguments safely (spaces / quotes)
  $argStr = ($ArgumentList | ForEach-Object {
    if ($_ -match '[\s"]') { '"' + ($_ -replace '"','\"') + '"' } else { $_ }
  }) -join ' '

  $psi = @{
    FilePath               = $FilePath
    ArgumentList           = $argStr
    RedirectStandardOutput = $StdOutPath
    RedirectStandardError  = $StdErrPath
    PassThru               = $true
  }
  if ($WorkingDirectory) { $psi.WorkingDirectory = $WorkingDirectory }

  $proc = Start-Process @psi

  $finished = $true
  try { $finished = $proc.WaitForExit($TimeoutSec * 1000) } catch { $finished = $false }

  if (-not $finished) {
    Write-Warning "Process '$FilePath' exceeded $TimeoutSec s; killing."
    try { Stop-Process -Id $proc.Id -Force } catch {}
  }

  if ($useTempErr -and (Test-Path -LiteralPath $StdErrPath)) {
    try {
      Add-Content -LiteralPath $StdOutPath -Value "`n--- STDERR ---`n"
      Get-Content -LiteralPath $StdErrPath | Add-Content -LiteralPath $StdOutPath
    } finally {
      Remove-Item -LiteralPath $StdErrPath -Force -ErrorAction SilentlyContinue
    }
  }

  if (-not $finished) { return $false }
  try { return ($proc.ExitCode -eq 0) } catch { return $false }
}

function Run-One {
  param(
    [Parameter(Mandatory)][string] $CityDir,
    [Parameter(Mandatory)][string] $Density
  )

  # Build the specific cfg for this density
  $cfgFile     = "osm{0}ps.sumocfg" -f $Density
  $srcCfgPath  = Join-Path $CityDir $cfgFile     # <- this is what Python will use
  $activeCfg   = Join-Path $CityDir "osm.sumocfg"

  if (-not (Test-Path $srcCfgPath)) {
    Write-Warning "Missing cfg: $srcCfgPath"
    return
  }

  # Optional: keep a copy as osm.sumocfg for other tools that expect that name
  Copy-Item -LiteralPath $srcCfgPath -Destination $activeCfg -Force

  $stamp = (Get-Date).ToString("yyyyMMdd_HHmmss")
  $name  = Split-Path -Leaf $CityDir
  $log_dijkstra = Join-Path $LogDir ("{0}_{1}Default_{2}.log" -f $name, $Density, $stamp)

  $scriptPath = Join-Path $Root "Dijkstra.py"
  if (-not (Test-Path $scriptPath)) {
    throw "Cannot find Dijkstra.py at: $scriptPath"
  }

  Write-Host "[$name $Density ps] Running Dijkstra.py with cfg '$cfgFile'"
  Stop-LeftoverSumo

  $args = @("-u", $scriptPath, $srcCfgPath)  # <<< pass cfg path to Python (argv[1])

  if ($Live) {
    # Run inline in the VS Code terminal and also tee to a log file
    Push-Location $CityDir
    try {
      Write-Host ">>> $Python $($args -join ' ')" -ForegroundColor Cyan
      # stream stdout/stderr to console AND save to log
      & $Python @args 2>&1 | Tee-Object -FilePath $log_dijkstra
      $ok = ($LASTEXITCODE -eq 0)
    } finally {
      Pop-Location
    }
  } else {
    # Detached with timeout + redirection to file only
    $ok = Invoke-ProcessWithTimeout `
      -FilePath $Python `
      -ArgumentList $args `
      -StdOutPath $log_dijkstra `
      -TimeoutSec $TimeoutSecPerRun `
      -WorkingDirectory $CityDir
  }

  if (-not $ok) {
    Write-Warning "Run failed (city=$name, density=$Density). See log: $log_dijkstra"
  }

  Stop-LeftoverSumo

  # Optional: summarize if SUMO printed "Simulation ended at time:"
  Write-LogSummary -LogPath $log_dijkstra
}

# -------- Main loop --------
foreach ($city in $Cities) {
  $cityDir = Join-Path $Root $city
  if (-not (Test-Path $cityDir)) {
    Write-Warning "Missing folder: $cityDir"
    continue
  }
  Write-Host "=== City: $city ==="
  foreach ($d in $Densities) {
    Write-Host "--- Density: $d ps ---"
    Run-One -CityDir $cityDir -Density $d
  }
}

Write-Host "Done. Logs in: $LogDir"
