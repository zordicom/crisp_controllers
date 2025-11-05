#!/bin/bash
# Fast build script for crisp_controllers with Pinocchio optimizations

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting optimized build for crisp_controllers...${NC}"
echo -e "${YELLOW}Build optimizations enabled:${NC}"
echo -e "  ✓ Explicit template instantiation for Pinocchio"
echo -e "  ✓ Unity/Jumbo builds"
echo -e "  ✓ Precompiled headers"
echo -e "  ✓ ccache compilation cache"

# Set number of parallel jobs (use all available cores)
JOBS=$(nproc)
echo -e "${YELLOW}Using $JOBS parallel jobs${NC}"

# Configure ccache for maximum efficiency
if command -v ccache &> /dev/null; then
    echo -e "${GREEN}Configuring ccache...${NC}"
    ccache -M 10G  # Set cache size to 10GB
    ccache -z      # Zero statistics
fi

# Build with optimized settings
echo -e "${GREEN}Building with optimized settings...${NC}"

# Check if gold linker is available
LINKER_FLAGS=""
if command -v ld.gold &> /dev/null; then
    echo -e "${YELLOW}Using gold linker for faster linking${NC}"
    LINKER_FLAGS="-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=gold"
fi

# For development builds (faster compilation, reasonable runtime performance)
colcon build \
    --packages-select crisp_controllers \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DUSE_PRECOMPILED_HEADERS=ON \
        -DUSE_UNITY_BUILD=ON \
        -DCMAKE_CXX_FLAGS="-pipe" \
        $LINKER_FLAGS \
    --parallel-workers $JOBS

# Print ccache statistics
if command -v ccache &> /dev/null; then
    echo -e "${GREEN}Build complete! ccache statistics:${NC}"
    ccache -s
fi

echo -e "${GREEN}Build finished!${NC}"