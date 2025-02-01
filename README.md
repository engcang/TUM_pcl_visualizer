<h2 align="center"><span>TUM PCL Visualizer</span></h2>

- Visualizing tool to check quality of the map with the estimated pose of your odometry / SLAM algorithm
    - It gets `TUM` format estimated pose - tx px py pz qx qy qz qw
    - It gets the `rosbag` file that used for pose estimation
    - It publishes the transformed pointcloud to estimated pose frame
    - It also publishes the estimated odometry in nav_msgs/Path

<br>

## Dependencies
+ `C++` `CMake`, `Eigen`
+ `ROS1`
+ `yaml-cpp`
    ```bash
    sudo apt install libyaml-cpp-dev
    ```
+ `livox_ros_driver` package for converting livox LiDAR data from many datasets
    + Specifically, `livox_ros_driver/CustomMsg`

<br>

## How to build and use
+ Get the code and then build
    ```bash
    cd ~/your_workspace/src
    git clone https://github.com/engcang/tum_pcl_visualizer

    cd ~/your_workspace
    catkin build -DCMAKE_BUILD_TYPE=Release
    ```
+ Run the code and watch the pointcloud with `Rviz`
    ```bash
    rosrun tum_pcl_visualizer tum_pcl_visualizer_node config_file bag_file_abs_path tum_csv_file_abs_path publish_hz max_t_diff_btw_pcl_and_pose time_offset
    ```

<br>

## TODO
- [ ] velodyne packets to sensor_msgs::PointCloud2
- [ ] `ROS2`

<br>

## LICENSE
+ This code is coded by myself and ChatGPT and hence it is free to use, distribute, etc.
+ I guess it is under BSD3 or MIT license.
