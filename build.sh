#!/bin/bash
set -e

cd "$(dirname "$0")"

# Build for the target platform
GOOS=${GOOS:-$(go env GOOS)}
GOARCH=${GOARCH:-$(go env GOARCH)}

echo "Building trossen module for $GOOS/$GOARCH..."

# Build the binary
go build -o trossen -ldflags="-s -w" .

# Create output directory
mkdir -p bin

# Create tarball with binary, run script, and 3D models
echo "Creating module tarball..."
tar -czvf bin/module.tar.gz \
    trossen \
    run.sh \
    arm/3d_models/

echo "Build complete: bin/module.tar.gz"
