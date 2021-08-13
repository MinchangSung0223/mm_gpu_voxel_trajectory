



# libfranka_gpu_voxel
```bash
rm -r CMakeCache.txt
bash build.sh
source export export_MODEL_PATH.sh
```

# 캘리브레이션 결과인 TBaseToCamera.txt 와  Target Position인 TargetPosition.txt를 작성해서 같은폴더에 위치시킨다.

```bash
./gvl_ompl_planner 0 0 0 0 0 0
```
