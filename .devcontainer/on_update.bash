#!/bin/bash
cd mtt_ws/
colcon build --symlink-install --base-paths mtt_ws --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
cd ..
