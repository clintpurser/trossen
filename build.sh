#!/bin/bash
set -e

cd "$(dirname "$0")"

# Build for the target platform
GOOS=${GOOS:-$(go env GOOS)}
GOARCH=${GOARCH:-$(go env GOARCH)}

echo "Building trossen module for $GOOS/$GOARCH..."

go build -o trossen -ldflags="-s -w" .

echo "Build complete: trossen"
