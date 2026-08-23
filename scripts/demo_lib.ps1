<#
.SYNOPSIS
    Shared process-management helpers for run_demo.ps1 / stop_demo.ps1.

.DESCRIPTION
    Every "find our process" function here identifies processes by BOTH their command line
    content (e.g. `serve_unity_bridge.py`, `uvicorn ... backend.main:app`, `vite`) AND that the
    command line references a path inside THIS repo checkout ($RepoRoot) -- never by process
    name alone (`python.exe`/`node.exe`), and never a blanket "kill everything on this port"
    without first checking it's actually one of ours. This is what keeps run_demo.ps1/
    stop_demo.ps1 from ever touching an unrelated Python/Node process elsewhere on the machine
    (e.g. a second, unrelated project's dev server, or a user's own Python REPL), even if such a
    process happens to be listening on one of this project's ports.
#>

function Get-RepoRoot {
    Split-Path -Parent $PSScriptRoot
}

function Get-ProcessesByCommandLine {
    <#
    .SYNOPSIS
        Returns Win32_Process rows for $ProcessName whose CommandLine matches every pattern in
        $Patterns (regex, ANDed together). Never throws -- CIM/WMI can be flaky; an empty result
        on error is treated the same as "found none", not a fatal condition, by every caller.
    #>
    param(
        [Parameter(Mandatory)] [string]$ProcessName,
        [Parameter(Mandatory)] [string[]]$Patterns
    )
    try {
        $rows = Get-CimInstance Win32_Process -Filter "Name='$ProcessName'" -ErrorAction Stop
    } catch {
        Write-Warning "Get-ProcessesByCommandLine: could not query processes ($($_.Exception.Message))."
        return @()
    }
    # NOTE: deliberately NOT named $matches -- that's PowerShell's automatic variable populated
    # by the `-match` operator used below; reusing the name here corrupts it mid-loop (each
    # `-match` call overwrites it with a regex-capture hashtable, so `+=` on a plain array then
    # fails with "A hash table can only be added to another hash table").
    $found = @()
    foreach ($row in $rows) {
        if (-not $row.CommandLine) { continue }
        $allMatch = $true
        foreach ($pat in $Patterns) {
            if ($row.CommandLine -notmatch $pat) { $allMatch = $false; break }
        }
        if ($allMatch) { $found += $row }
    }
    # Deliberately a plain (uncommaed) return -- PowerShell unwraps a 0- or 1-element array as it
    # flows through the pipeline/each pass-through function (Get-BridgeProcesses etc. just relay
    # this call's output), so by the time it reaches a caller it may be $null, a bare CimInstance,
    # or a real array depending on the count. Every caller MUST wrap the call in `@(...)` at the
    # point it needs a reliable array/.Count (already done in run_demo.ps1/stop_demo.ps1) -- doing
    # it here too would double-wrap (`@(...)` around an already-array-typed single pipeline object
    # produces a 1-element array THAT CONTAINS an array), which is a real bug this project hit
    # while testing: a 2-element match came back as one "process" whose .ProcessId/.CommandLine
    # were themselves 2-element arrays, string-interpolated as space-joined garbage like
    # "PID 253212 259076".
    return $found
}

function Get-BridgeProcesses {
    param([Parameter(Mandatory)] [string]$RepoRoot)
    $escaped = [regex]::Escape($RepoRoot)
    Get-ProcessesByCommandLine -ProcessName "python.exe" -Patterns @($escaped, "serve_unity_bridge\.py")
}

function Get-BackendProcesses {
    param([Parameter(Mandatory)] [string]$RepoRoot)
    $escaped = [regex]::Escape($RepoRoot)
    Get-ProcessesByCommandLine -ProcessName "python.exe" -Patterns @($escaped, "uvicorn", "backend\.main:app")
}

function Get-DashboardProcesses {
    param([Parameter(Mandatory)] [string]$DashboardDir)
    $escaped = [regex]::Escape($DashboardDir)
    # `npm run dev` (via cmd.exe /c) spawns a node.exe running vite -- match on the dashboard's
    # own path being present in the node.exe command line (e.g. .../cloud/dashboard/node_modules/
    # vite/bin/vite.js or the vite config path), not just "any vite".
    Get-ProcessesByCommandLine -ProcessName "node.exe" -Patterns @($escaped)
}

function Stop-ProcessRowSafely {
    param(
        [Parameter(Mandatory)] $ProcessRow,
        [Parameter(Mandatory)] [string]$Label
    )
    $procId = $ProcessRow.ProcessId
    Write-Host "  Stopping $Label (PID $procId): $($ProcessRow.CommandLine)" -ForegroundColor Yellow
    try {
        Stop-Process -Id $procId -Force -ErrorAction Stop
    } catch {
        Write-Warning "  Could not stop PID $procId ($($_.Exception.Message)) -- it may have already exited."
    }
}

function Test-PortListening {
    param([Parameter(Mandatory)] [int]$Port)
    $conns = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
    return [bool]$conns
}

function Wait-PortFree {
    param(
        [Parameter(Mandatory)] [int]$Port,
        [int]$TimeoutSec = 10
    )
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (-not (Test-PortListening -Port $Port)) { return $true }
        Start-Sleep -Milliseconds 300
    }
    return -not (Test-PortListening -Port $Port)
}

function Wait-PortListening {
    param(
        [Parameter(Mandatory)] [int]$Port,
        [int]$TimeoutSec = 20
    )
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Test-PortListening -Port $Port) { return $true }
        Start-Sleep -Milliseconds 300
    }
    return Test-PortListening -Port $Port
}

function Get-PortOwnerDescription {
    <# Best-effort "what's on this port" description, used only for diagnostic messages -- never
       to decide whether to kill it. #>
    param([Parameter(Mandatory)] [int]$Port)
    $conns = Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue
    if (-not $conns) { return "nothing" }
    $descs = @()
    foreach ($c in $conns) {
        $proc = Get-Process -Id $c.OwningProcess -ErrorAction SilentlyContinue
        if ($proc) {
            $descs += "$($proc.ProcessName) (PID $($c.OwningProcess))"
        } else {
            $descs += "PID $($c.OwningProcess)"
        }
    }
    return ($descs -join ", ")
}
