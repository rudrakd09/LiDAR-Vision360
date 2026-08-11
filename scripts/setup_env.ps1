# Sets up the Python virtual environment for perception development on Windows.
# Run from the repository root:  .\scripts\setup_env.ps1

$ErrorActionPreference = "Stop"

python -m venv .venv
# Use `python -m pip` rather than pip.exe directly: pip.exe prints a self-referential notice to
# stderr that PowerShell wraps as a terminating NativeCommandError under $ErrorActionPreference
# = "Stop", aborting the script even though the install itself succeeded.
& .\.venv\Scripts\python.exe -m pip install --upgrade pip
& .\.venv\Scripts\python.exe -m pip install -e ".\perception[dev]"
& .\.venv\Scripts\python.exe -m pip install -e ".\simulator[dev,viz]"

if (-not (Test-Path ".env")) {
    Copy-Item ".env.example" ".env"
    Write-Host "Created .env from .env.example -- review and adjust values as needed."
}

Write-Host ""
Write-Host "Done. Activate the environment with:"
Write-Host "    .\.venv\Scripts\Activate.ps1"
Write-Host "Then run tests with:"
Write-Host "    pytest perception/tests"
Write-Host "    pytest simulator/tests"
