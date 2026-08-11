# Sets up the Python virtual environment for perception development on Windows.
# Run from the repository root:  .\scripts\setup_env.ps1

$ErrorActionPreference = "Stop"

python -m venv .venv
& .\.venv\Scripts\pip.exe install --upgrade pip
& .\.venv\Scripts\pip.exe install -e ".\perception[dev]"

if (-not (Test-Path ".env")) {
    Copy-Item ".env.example" ".env"
    Write-Host "Created .env from .env.example -- review and adjust values as needed."
}

Write-Host ""
Write-Host "Done. Activate the environment with:"
Write-Host "    .\.venv\Scripts\Activate.ps1"
Write-Host "Then run tests with:"
Write-Host "    pytest perception/tests"
