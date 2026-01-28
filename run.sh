#!/bin/bash
# Local module runner for development

cd "$(dirname "$0")"

# Use the local venv if it exists, otherwise create one
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
    .venv/bin/pip install -r requirements.txt
fi

# Run the module
exec .venv/bin/python src/main.py "$@"
