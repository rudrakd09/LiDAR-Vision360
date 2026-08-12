<#
.SYNOPSIS
    Single-command launcher for the full LiDAR-Vision360 synthetic demo: Python perception bridge
    (Unity + backend source) + local cloud backend (FastAPI) + dashboard (Vite dev server).

.DESCRIPTION
    Starts three processes, each in its own visible window (so their logs stay watchable, the same
    as running three terminals by hand):
      1. scripts/serve_unity_bridge.py  -- the ONE synthetic-data source; Unity and the backend
         both connect to its port 5006 independently (see docs/cloud.md "Architecture" -- this is
         not Python -> Unity -> Dashboard, both are direct clients of the same broadcast).
      2. cloud/backend (uvicorn)         -- ingests that same stream, serves REST + WebSocket.
      3. cloud/dashboard (npm run dev)   -- the live web dashboard.

    Unity itself is started separately (open the project in the Editor and press Play) -- this
    script only starts the Python/backend/dashboard side, per docs/unity.md.

.PARAMETER Scenario
    Simulator scenario id (default: 08_approaching_obstacle, this project's primary demo scenario).

.PARAMETER Rate
    Target scans/sec (default: 10).

.PARAMETER VehicleSpeed
    Ego vehicle forward speed, m/s (default: 0 -- stationary vehicle, approaching obstacle).

.PARAMETER SkipDashboardInstall
    Skip `npm install` even if cloud/dashboard/node_modules is missing (assumes it's already been
    run).

.EXAMPLE
    ./scripts/run_demo.ps1
    ./scripts/run_demo.ps1 -Scenario 07_moving_crossing -Rate 10
#>
param(
    [string]$Scenario = "08_approaching_obstacle",
    [double]$Rate = 10,
    [double]$VehicleSpeed = 0,
    [switch]$SkipDashboardInstall
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$VenvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $VenvPython)) {
    Write-Warning "No .venv found at $VenvPython -- falling back to 'python' on PATH."
    $VenvPython = "python"
}

$BackendSrc = Join-Path $RepoRoot "cloud\backend\src"
$DashboardDir = Join-Path $RepoRoot "cloud\dashboard"
$DashboardNodeModules = Join-Path $DashboardDir "node_modules"

Write-Host "=== LiDAR-Vision360 demo launcher ===" -ForegroundColor Cyan
Write-Host "Scenario:      $Scenario"
Write-Host "Rate:          $Rate Hz"
Write-Host "Vehicle speed: $VehicleSpeed m/s"
Write-Host ""

# --- 1. Perception bridge (the single source of synthetic data) ---
Write-Host "[1/3] Starting perception bridge (ports 5005 raw / 5006 structured JSON)..." -ForegroundColor Yellow
Start-Process -FilePath $VenvPython `
    -ArgumentList @(
        (Join-Path $RepoRoot "scripts\serve_unity_bridge.py"),
        "--scenario", $Scenario, "--rate", $Rate, "--vehicle-speed", $VehicleSpeed
    ) `
    -WorkingDirectory $RepoRoot `
    -WindowStyle Normal

Start-Sleep -Seconds 2

# --- 2. Cloud backend ---
Write-Host "[2/3] Starting backend (http://localhost:8000)..." -ForegroundColor Yellow
Start-Process -FilePath $VenvPython `
    -ArgumentList @(
        "-m", "uvicorn", "backend.main:app",
        "--app-dir", $BackendSrc, "--host", "0.0.0.0", "--port", "8000"
    ) `
    -WorkingDirectory $RepoRoot `
    -WindowStyle Normal

Start-Sleep -Seconds 2

# --- 3. Dashboard ---
if (-not (Test-Path $DashboardNodeModules) -and -not $SkipDashboardInstall) {
    Write-Host "[3/3] Installing dashboard dependencies (first run)..." -ForegroundColor Yellow
    Push-Location $DashboardDir
    try {
        & npm install
        if ($LASTEXITCODE -ne 0) { throw "npm install failed with exit code $LASTEXITCODE" }
    } finally {
        Pop-Location
    }
}

Write-Host "[3/3] Starting dashboard (http://localhost:5173)..." -ForegroundColor Yellow
# `npm` resolves to npm.ps1/npm.cmd (a shell shim), not a directly-launchable .exe -- Start-Process
# -FilePath "npm" silently fails to actually start it. Route through cmd.exe /c instead, which
# resolves PATHEXT shims (.cmd/.ps1/.exe) the same way typing `npm run dev` at a prompt would.
Start-Process -FilePath "cmd.exe" -ArgumentList @("/c", "npm run dev") -WorkingDirectory $DashboardDir -WindowStyle Normal

Write-Host ""
Write-Host "=== All three processes started in their own windows ===" -ForegroundColor Cyan
Write-Host "  Perception bridge:  raw 127.0.0.1:5005, structured JSON 127.0.0.1:5006 (Unity connects here)"
Write-Host "  Backend API:        http://localhost:8000  (try http://localhost:8000/api/health)"
Write-Host "  Dashboard:          http://localhost:5173"
Write-Host ""
Write-Host "To also see the Unity digital twin: open unity/LiDARVision360 in the Unity Editor and press Play"
Write-Host "(set LidarInputManager.mode = StructuredJsonTcp -- see docs/unity.md 'Setup')."
Write-Host ""
Write-Host "Run scripts/stop_demo.ps1 to stop all three when done."
