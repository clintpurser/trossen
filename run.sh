#!/bin/bash
# Local module runner for development

cd "$(dirname "$0")"

# Run the Go binary
exec ./trossen "$@"
