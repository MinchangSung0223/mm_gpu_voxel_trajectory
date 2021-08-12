#!/bin/bash
rm -r CMakeCache.txt
cp TemplateVoxelMap.hpp /root/workspace/gpu-voxels/packages/gpu_voxels/src/gpu_voxels/voxelmap/
cmake . -D icl_core_DIR=~/workspace/gpu-voxels/build/packages/icl_core/ -D gpu_voxels_DIR=~/workspace/gpu-voxels/build/packages/gpu_voxels
make -j16
export GPU_VOXELS_MODEL_PATH=.
