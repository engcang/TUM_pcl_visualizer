<h2 align="center"><span>LODESTAR: A Robust LiDAR-Inertial Odometry<br>Using Degeneracy-Aware Adaptive Schmidt Filter<br>with Sliding Window and Data Exploitation</span></h2>

<br>

## Dependencies
+ `C++` >= 17, `CMake` >= 3.0, `Eigen` >= 3.2
+ `ROS1`
+ `tbb`
    ```bash
    sudo apt install libtbb-dev
    ```

<br>

## How to build
+ Get the code and then build
    ```bash
    cd ~/your_workspace/src
    git clone https://github.com/engcang/Lodestar --recursive

    cd ~/your_workspace
    catkin build -DCMAKE_BUILD_TYPE=Release
    ```

<br>

## TODO
- [ ] `ROS2`

<br>

## LICENSE
+ This version of repository is based on [FAST-LIO](https://github.com/hku-mars/FAST_LIO) and hence following `GPL-2.0`
+ `LODESTAR` itself is following the license:
    ```markdown
    Copyright (c) 2025, EungChang-Mason-Lee
    All rights reserved.

    Redistribution and use in source and binary forms, ONLY WITHOUT
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

    4. Redistributions or use in any forms are NOT FOR ANY METHOD OF COMMERCIAL USE.
    ```
