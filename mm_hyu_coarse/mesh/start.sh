#!/bin/bash
./voxelize.sh FT_sensor.stl 50;
./voxelize.sh FT_sensor1.stl 50;
./voxelize.sh FT_sensor2.stl 50;
./voxelize.sh SPA_Base_Link_.stl 50;
./voxelize.sh SPA_Link_01_.stl 50;
./voxelize.sh SPA_Link_02_.stl 50;
./voxelize.sh SPA_Link_03_.stl 50;
./voxelize.sh SPA_Link_04_.stl 50;
./voxelize.sh SPA_Link_05_.stl 50;
./voxelize.sh SPA_Link_06_.stl 50;
./voxelize.sh base_profile.stl 50;
./voxelize.sh baterry.stl 50;
./voxelize.sh battery.stl 50;
./voxelize.sh mobile_base.stl 50;
./voxelize.sh wheel.stl 50;
cp *.binvox ../../
