<#
.SYNOPSIS
    Reliably stops the processes scripts/run_demo.ps1 started -- identified by command line
    (this repo's path + the specific script/module each one runs), never by port or process name
    alone, so it can never take down an unrelated python.exe/node.exe elsewhere on the machine
    (e.g. a second, unrelated project's dev server on the same port).

.DESCRIPTION
    1. Stops the bridge (scripts/serve_unity_bridge.py).
    2. Stops the backend (uvicorn backend.main:app), unless -KeepBackend.
    3. Stops the dashboard (vite dev server), unless -KeepDashboard.
    4. Verifies each port it stopped is actually released afterwards.

    Safe to run even if only some of the three are still up.

.PARAMETER KeepBackend
    Leave the backend running (e.g. you're about to switch scenarios and don't want to pay
    uvicorn's startup cost again -- run_demo.ps1 will retain it automatically next time).

.PARAMETER KeepDashboard
    Leave the dashboard running, same reasoning.
#>
param(
    [switch]$KeepBackend,
    [switch]$KeepDashboard
)

$ErrorActionPreference = "Continue"

. (Join-Path $PSScriptRoot "demo_lib.ps1")

$RepoRoot = Get-RepoRoot
$DashboardDir = Join-Path $RepoRoot "cloud\dashboard"

function Stop-Group {
    param(
        [string]$Label,
        [array]$Processes,
        [int]$Port
    )
    if ($Processes.Count -eq 0) {
        Write-Host "$Label : nothing running." -ForegroundColor DarkGray
        return
    }
    foreach ($p in $Processes) {
        Stop-ProcessRowSafely -ProcessRow $p -Label $Label
    }
    if (Wait-PortFree -Port $Port -TimeoutSec 10) {
        Write-Host "$Label : stopped, port $Port released." -ForegroundColor Green
    } else {
        $owner = Get-PortOwnerDescription -Port $Port
        Write-Warning "$Label : port $Port still shows a listener after stopping ($owner) -- may be a different, unrelated process; not touching it."
    }
}

Write-Host "=== Stopping LiDAR-Vision360 demo ===" -ForegroundColor Cyan

# --- 1. Bridge ---
Write-Host "[1/4] Bridge (serve_unity_bridge.py)..." -ForegroundColor Yellow
$bridges = @(Get-BridgeProcesses -RepoRoot $RepoRoot)
Stop-Group -Label "Bridge" -Processes $bridges -Port 5006
if (-not (Wait-PortFree -Port 5005 -TimeoutSec 5)) {
    $owner = Get-PortOwnerDescription -Port 5005
    Write-Warning "Port 5005 still shows a listener ($owner) -- may be unrelated; not touching it."
} else {
    Write-Host "Port 5005 : released." -ForegroundColor DarkGray
}

# --- 2. Backend ---
Write-Host "[2/4] Backend (uvicorn backend.main:app)..." -ForegroundColor Yellow
if ($KeepBackend) {
    Write-Host "  -KeepBackend set -- leaving it running." -ForegroundColor DarkGray
} else {
    $backends = @(Get-BackendProcesses -RepoRoot $RepoRoot)
    Stop-Group -Label "Backend" -Processes $backends -Port 8000
}

# --- 3. Dashboard ---
Write-Host "[3/4] Dashboard (vite dev server)..." -ForegroundColor Yellow
if ($KeepDashboard) {
    Write-Host "  -KeepDashboard set -- leaving it running." -ForegroundColor DarkGray
} else {
    $dashboards = @(Get-DashboardProcesses -DashboardDir $DashboardDir)
    Stop-Group -Label "Dashboard" -Processes $dashboards -Port 5173
}

# --- 4. Final port verification ---
Write-Host "[4/4] Final port check..." -ForegroundColor Yellow
$expectedFree = @(5005, 5006)
if (-not $KeepBackend) { $expectedFree += 8000 }
if (-not $KeepDashboard) { $expectedFree += 5173 }
$allFree = $true
foreach ($port in $expectedFree) {
    if (Test-PortListening -Port $port) {
        $owner = Get-PortOwnerDescription -Port $port
        Write-Warning "Port $port : still listening ($owner)."
        $allFree = $false
    } else {
        Write-Host "Port $port : free." -ForegroundColor DarkGray
    }
}

Write-Host ""
if ($allFree) {
    Write-Host "Done -- all targeted ports released." -ForegroundColor Cyan
} else {
    Write-Host "Done -- see warnings above for any port still in use." -ForegroundColor Yellow
}
