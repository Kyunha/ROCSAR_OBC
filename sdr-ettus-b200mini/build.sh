#!/bin/bash
# chmod +x build.sh

TARGET="connect"

# Remove previous executable if it exists
if [ -f "$TARGET" ]; then
    echo "Removing old $TARGET..."
    rm -f "$TARGET"
fi

# Build
echo "Building..."
make

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo "Build successful -> ./$TARGET"
else
    echo "Build failed."
    exit 1
fi