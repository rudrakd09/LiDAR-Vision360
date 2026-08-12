<#
.SYNOPSIS
    Stops the three processes scripts/run_demo.ps1 started, by port -- not a blanket
    Stop-Process on every python.exe/node.exe, which could kill unrelated processes on the
    machine.

.DESCRIPTION
    Finds whatever process (if any) is currently listening on 5005 (raw), 5006 (structured JSON),
    8000 (backend API), and 5173 (dashboard dev server), and stops it. Safe to run even if only
    some of the three are still up (e.g. you already Ctrl+C'd one window).
#>

$ErrorActionPreference = "Continue"
$Ports = @(5005, 5006, 8000, 5173)

foreach ($port in $Ports) {
    $connections = Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue
    if (-not $connections) {
        Write-Host "Port $port : nothing listening." -ForegroundColor DarkGray
        continue
    }
    foreach ($conn in $connections) {
        $processId = $conn.OwningProcess
        $proc = Get-Process -Id $processId -ErrorAction SilentlyContinue
        if ($proc) {
            Write-Host "Port $port : stopping $($proc.ProcessName) (PID $processId)." -ForegroundColor Yellow
            Stop-Process -Id $processId -Force -ErrorAction SilentlyContinue
        }
    }
}

Write-Host "Done." -ForegroundColor Cyan
