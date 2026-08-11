#!/usr/bin/env bash
# Sets up the Python virtual environment for perception development on macOS/Linux.
# Run from the repository root: ./scripts/setup_env.sh
set -euo pipefail

python3 -m venv .venv
./.venv/bin/pip install --upgrade pip
./.venv/bin/pip install -e "./perception[dev]"
./.venv/bin/pip install -e "./simulator[dev,viz]"

if [ ! -f .env ]; then
    cp .env.example .env
    echo "Created .env from .env.example -- review and adjust values as needed."
fi

echo ""
echo "Done. Activate the environment with:"
echo "    source .venv/bin/activate"
echo "Then run tests with:"
echo "    pytest perception/tests"
echo "    pytest simulator/tests"
