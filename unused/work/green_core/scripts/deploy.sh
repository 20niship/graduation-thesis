#!/bin/bash
mkdir -p ~/hr4c_core/bin
mkdir -p ~/hr4c_core/scripts
cp ../cmake-build-debug/config.yaml ~/hr4c_core/bin
cp ../cmake-build-debug/green_core_rt ~/hr4c_core/bin/hr4c_core_rt
cp ./hr4c_startup.sh ~/hr4c_core/scripts
