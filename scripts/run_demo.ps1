<#
.SYNOPSIS
    Single-command, reliable launcher for the full LiDAR-Vision360 synthetic demo: Python
    perception bridge (Unity + backend source) + local cloud backend (FastAPI) + dashboard (Vite
    dev server) -- fixed to never leave a stale `serve_unity_bridge.py` holding ports 5005/5006,
    which is what previously caused `WinError 10048: Address already in use` on a second run.

.DESCRIPTION
    Runs, in order (see scripts/demo_lib.ps1 for the process-identification helpers):
      1. Detect existing project bridge process(es) -- by command line, not just by port/name.
      2. Stop ONLY those (never an unrelated python.exe/node.exe elsewhere on the machine).
      3. Verify ports 5005 and 5006 are actually free before starting a new one.
      4. Start exactly one new bridge for -Scenario, and confirm it actually bound both ports
         (rather than, say, silently having crashed on an import error).
      5. Start the backend, or retain it if an already-running, healthy instance owned by this
         repo is found on port 8000 (a fresh bridge process re-attaches to it automatically --
         `cloud/backend`'s ingestor reconnects, it does not need to be restarted per scenario).
      6. Start the dashboard, or retain it the same way on port 5173.
      7-11. Verify /health, /debug/live-frame, source_id, the /ws/live WebSocket, and that
         scenario frames are actually changing scan-to-scan -- via scripts/verify_demo.py, so a
         failure here is a clear, actionable error instead of "looks fine, dashboard is just
         blank."

.PARAMETER Scenario
    Simulator scenario id (default: 08_approaching_obstacle). See simulator/scenarios/*.json.

.PARAMETER Rate
    Target scans/sec (default: 10).

.PARAMETER VehicleSpeed
    Ego vehicle forward speed, m/s (default: 0 -- stationary vehicle, approaching obstacle).

.PARAMETER SkipDashboardInstall
    Skip `npm install` even if cloud/dashboard/node_modules is missing.

.PARAMETER Hidden
    Start the bridge/backend/dashboard with hidden windows and log to scripts/.demo_logs/*.log
    instead of visible terminal windows -- for unattended/scripted runs (e.g. iterating over all
    10 scenarios back to back). Default is visible windows, same as before.

.PARAMETER RestartBackend
.PARAMETER RestartDashboard
    Force-restart the backend/dashboard even if a healthy instance owned by this repo is already
    running, instead of retaining it.

.EXAMPLE
    ./scripts/run_demo.ps1 -Scenario 07_moving_crossing -Rate 10
    ./scripts/run_demo.ps1 -Scenario 09_noisy_lidar -Hidden
#>
param(
    [string]$Scenario = "08_approaching_obstacle",
    [double]$Rate = 10,
    [double]$VehicleSpeed = 0,
    [switch]$SkipDashboardInstall,
    [switch]$Hidden,
    [switch]$RestartBackend,
    [switch]$RestartDashboard
)

$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "demo_lib.ps1")

$RepoRoot = Get-RepoRoot
$VenvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $VenvPython)) {
    Write-Warning "No .venv found at $VenvPython -- falling back to 'python' on PATH."
    $VenvPython = "python"
}

$BackendSrc = Join-Path $RepoRoot "cloud\backend\src"
$DashboardDir = Join-Path $RepoRoot "cloud\dashboard"
$DashboardNodeModules = Join-Path $DashboardDir "node_modules"
$VerifyScript = Join-Path $PSScriptRoot "verify_demo.py"

$LogDir = Join-Path $PSScriptRoot ".demo_logs"
if ($Hidden -and -not (Test-Path $LogDir)) {
    New-Item -ItemType Directory -Path $LogDir -Force | Out-Null
}

$BackendUrl = "http://localhost:8000"
$WsUrl = "ws://localhost:8000/ws/live"
$ExpectedSourceId = "simulated:$Scenario"

$WindowStyle = if ($Hidden) { "Hidden" } else { "Normal" }

function Invoke-Verify {
    param([string[]]$VerifyArgs)
    $output = & $VenvPython $VerifyScript @VerifyArgs 2>&1
    $exit = $LASTEXITCODE
    return @{ Output = ($output -join "`n"); ExitCode = $exit }
}

function Assert-Verify {
    param([string]$StepLabel, [string[]]$VerifyArgs)
    Write-Host "  $StepLabel..." -ForegroundColor Yellow -NoNewline
    $r = Invoke-Verify -VerifyArgs $VerifyArgs
    if ($r.ExitCode -ne 0) {
        Write-Host " FAILED" -ForegroundColor Red
        Write-Host $r.Output
        throw "$StepLabel failed."
    }
    Write-Host " OK" -ForegroundColor Green
    Write-Host "    $($r.Output)" -ForegroundColor DarkGray
    return $r.Output
}

Write-Host "=== LiDAR-Vision360 demo launcher ===" -ForegroundColor Cyan
Write-Host "Scenario:      $Scenario"
Write-Host "Rate:          $Rate Hz"
Write-Host "Vehicle speed: $VehicleSpeed m/s"
Write-Host ""

# --- 1/2. Detect and stop only OUR existing bridge process(es) ---
Write-Host "[1/11] Detecting existing project bridge process(es)..." -ForegroundColor Yellow
$existingBridges = @(Get-BridgeProcesses -RepoRoot $RepoRoot)
if ($existingBridges.Count -eq 0) {
    Write-Host "  None found." -ForegroundColor DarkGray
} else {
    Write-Host "[2/11] Stopping $($existingBridges.Count) old bridge process(es)..." -ForegroundColor Yellow
    foreach ($p in $existingBridges) {
        Stop-ProcessRowSafely -ProcessRow $p -Label "old serve_unity_bridge.py"
    }
}

# --- 3. Verify 5005 and 5006 are free ---
Write-Host "[3/11] Verifying ports 5005/5006 are free..." -ForegroundColor Yellow
foreach ($port in @(5005, 5006)) {
    if (-not (Wait-PortFree -Port $port -TimeoutSec 10)) {
        $owner = Get-PortOwnerDescription -Port $port
        throw "Port $port is still in use (by: $owner) after stopping this repo's bridge process(es). " +
              "This is NOT one of ours -- refusing to kill it. Stop it manually, or use a different port " +
              "(LIDAR_LIDAR_STREAMING_RAW_PORT / LIDAR_LIDAR_STREAMING_JSON_PORT), then re-run."
    }
    Write-Host "  Port $port : free." -ForegroundColor DarkGray
}

# --- 4. Start exactly one bridge ---
Write-Host "[4/11] Starting perception bridge (scenario '$Scenario')..." -ForegroundColor Yellow
$bridgeArgs = @(
    (Join-Path $RepoRoot "scripts\serve_unity_bridge.py"),
    "--scenario", $Scenario, "--rate", $Rate, "--vehicle-speed", $VehicleSpeed
)
$bridgeStartArgs = @{
    FilePath         = $VenvPython
    ArgumentList     = $bridgeArgs
    WorkingDirectory = $RepoRoot
    WindowStyle      = $WindowStyle
    PassThru         = $true
}
if ($Hidden) {
    $bridgeStartArgs["RedirectStandardOutput"] = Join-Path $LogDir "bridge.out.log"
    $bridgeStartArgs["RedirectStandardError"] = Join-Path $LogDir "bridge.err.log"
}
$bridgeProc = Start-Process @bridgeStartArgs

Start-Sleep -Seconds 1
if ($bridgeProc.HasExited) {
    $hint = if ($Hidden) { " See $LogDir\bridge.err.log." } else { "" }
    throw "Bridge process exited immediately (exit code $($bridgeProc.ExitCode)).$hint"
}
if (-not (Wait-PortListening -Port 5006 -TimeoutSec 15) -or -not (Wait-PortListening -Port 5005 -TimeoutSec 5)) {
    throw "Bridge process (PID $($bridgeProc.Id)) did not bind ports 5005/5006 within the timeout."
}
Write-Host "  Bridge PID $($bridgeProc.Id) listening on 5005 (raw) / 5006 (JSON)." -ForegroundColor Green

# --- 5. Start/retain backend ---
Write-Host "[5/11] Backend (http://localhost:8000)..." -ForegroundColor Yellow
$existingBackends = @(Get-BackendProcesses -RepoRoot $RepoRoot)
$backendHealthy = $false
if ($existingBackends.Count -gt 0 -and -not $RestartBackend) {
    try {
        $h = Invoke-RestMethod -Uri "$BackendUrl/health" -TimeoutSec 3
        if ($h.status -eq "ok") { $backendHealthy = $true }
    } catch { $backendHealthy = $false }
}
if ($backendHealthy) {
    Write-Host "  Already running and healthy (PID $($existingBackends[0].ProcessId)) -- retaining." -ForegroundColor DarkGray
} else {
    if ($existingBackends.Count -gt 0) {
        Write-Host "  Existing backend process found but not healthy/-RestartBackend set -- restarting." -ForegroundColor Yellow
        foreach ($p in $existingBackends) { Stop-ProcessRowSafely -ProcessRow $p -Label "old backend" }
        Wait-PortFree -Port 8000 -TimeoutSec 10 | Out-Null
    } elseif (Test-PortListening -Port 8000) {
        $owner = Get-PortOwnerDescription -Port 8000
        throw "Port 8000 is occupied by a process that is NOT this repo's backend ($owner). Refusing to kill it -- free the port manually and re-run."
    }
    $backendArgs = @("-m", "uvicorn", "backend.main:app", "--app-dir", $BackendSrc, "--host", "0.0.0.0", "--port", "8000")
    $backendStartArgs = @{
        FilePath         = $VenvPython
        ArgumentList     = $backendArgs
        WorkingDirectory = $RepoRoot
        WindowStyle      = $WindowStyle
        PassThru         = $true
    }
    if ($Hidden) {
        $backendStartArgs["RedirectStandardOutput"] = Join-Path $LogDir "backend.out.log"
        $backendStartArgs["RedirectStandardError"] = Join-Path $LogDir "backend.err.log"
    }
    $backendProc = Start-Process @backendStartArgs
    if (-not (Wait-PortListening -Port 8000 -TimeoutSec 20)) {
        $hint = if ($Hidden) { " See $LogDir\backend.err.log." } else { "" }
        throw "Backend (PID $($backendProc.Id)) did not start listening on port 8000 within the timeout.$hint"
    }
    Write-Host "  Started (PID $($backendProc.Id))." -ForegroundColor Green
}

# --- 6. Start/retain dashboard ---
Write-Host "[6/11] Dashboard (http://localhost:5173)..." -ForegroundColor Yellow
if (-not (Test-Path $DashboardNodeModules) -and -not $SkipDashboardInstall) {
    Write-Host "  Installing dashboard dependencies (first run)..." -ForegroundColor Yellow
    Push-Location $DashboardDir
    try {
        & npm install
        if ($LASTEXITCODE -ne 0) { throw "npm install failed with exit code $LASTEXITCODE" }
    } finally {
        Pop-Location
    }
}
$existingDashboards = @(Get-DashboardProcesses -DashboardDir $DashboardDir)
$dashboardUp = $false
if ($existingDashboards.Count -gt 0 -and -not $RestartDashboard -and (Test-PortListening -Port 5173)) {
    $dashboardUp = $true
}
if ($dashboardUp) {
    Write-Host "  Already running (PID $($existingDashboards[0].ProcessId)) -- retaining." -ForegroundColor DarkGray
} else {
    if ($existingDashboards.Count -gt 0) {
        Write-Host "  Existing dashboard process found but not up/-RestartDashboard set -- restarting." -ForegroundColor Yellow
        foreach ($p in $existingDashboards) { Stop-ProcessRowSafely -ProcessRow $p -Label "old dashboard" }
        Wait-PortFree -Port 5173 -TimeoutSec 10 | Out-Null
    } elseif (Test-PortListening -Port 5173) {
        $owner = Get-PortOwnerDescription -Port 5173
        throw "Port 5173 is occupied by a process that is NOT this repo's dashboard ($owner). Refusing to kill it -- free the port manually and re-run."
    }
    # `npm` resolves to npm.ps1/npm.cmd (a shell shim), not a directly-launchable .exe --
    # Start-Process -FilePath "npm" silently fails to actually start it. Route through cmd.exe /c.
    $dashboardStartArgs = @{
        FilePath         = "cmd.exe"
        ArgumentList     = @("/c", "npm run dev")
        WorkingDirectory = $DashboardDir
        WindowStyle      = $WindowStyle
        PassThru         = $true
    }
    if ($Hidden) {
        $dashboardStartArgs["RedirectStandardOutput"] = Join-Path $LogDir "dashboard.out.log"
        $dashboardStartArgs["RedirectStandardError"] = Join-Path $LogDir "dashboard.err.log"
    }
    $dashboardProc = Start-Process @dashboardStartArgs
    if (-not (Wait-PortListening -Port 5173 -TimeoutSec 30)) {
        $hint = if ($Hidden) { " See $LogDir\dashboard.err.log." } else { "" }
        throw "Dashboard did not start listening on port 5173 within the timeout.$hint"
    }
    Write-Host "  Started." -ForegroundColor Green
}

# --- 7-11. Verification ---
Write-Host ""
Write-Host "Verifying end-to-end..." -ForegroundColor Cyan
Assert-Verify -StepLabel "[7/11] /health" -VerifyArgs @("health", "--base-url", $BackendUrl, "--timeout", "20") | Out-Null
Assert-Verify -StepLabel "[8/11] /debug/live-frame" -VerifyArgs @("live-frame", "--base-url", $BackendUrl, "--timeout", "20") | Out-Null
Assert-Verify -StepLabel "[9/11] source_id == $ExpectedSourceId" -VerifyArgs @("source-id", "--base-url", $BackendUrl, "--expect", $ExpectedSourceId, "--timeout", "15") | Out-Null
Assert-Verify -StepLabel "[10/11] WebSocket /ws/live" -VerifyArgs @("websocket", "--ws-url", $WsUrl, "--timeout", "10") | Out-Null
Assert-Verify -StepLabel "[11/11] scenario frames changing" -VerifyArgs @("frames-changing", "--base-url", $BackendUrl, "--samples", "3", "--interval", "1.2") | Out-Null

Write-Host ""
Write-Host "=== Demo up and verified ===" -ForegroundColor Cyan
Write-Host "  Perception bridge:  raw 127.0.0.1:5005, structured JSON 127.0.0.1:5006 (PID $($bridgeProc.Id))"
Write-Host "  Backend API:        $BackendUrl  (try $BackendUrl/health)"
Write-Host "  Dashboard:          http://localhost:5173"
Write-Host ""
Write-Host "To also see the Unity digital twin: open unity/LiDARVision360 in the Unity Editor and press Play"
Write-Host "(set LidarInputManager.mode = StructuredJsonTcp -- see docs/unity.md 'Setup')."
Write-Host ""
Write-Host "Run scripts/stop_demo.ps1 to stop when done."
