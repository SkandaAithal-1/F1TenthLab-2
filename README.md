# F1TenthLab-2

This ROS2 package implements a simple Automatic Emergency Breaking with iTTC.

$$iTTC = \frac{r}{\{-\dot{r}\}_+}$$

The $-\dot{r}$ here was calculated as $v_x cos(\theta)$  
$r$ is obtained from the range data from the topic `scan`  
$v_x$ is obtained from the topic `/ego_racecar/odom`

For braking, speed of 0 is published to the topic `drive`

The threshold for braking is set using a parameter `TTCThreshold` which can be changed in the launch file

## Demonstration

file:///home/skanda/git/demo.mp4


## Build instruction

Place this package in a colcon workspace. Then

```
colcon build
```

To build only this package use -

```
colcon build --packages-select safety_node
```

## Run instruction

Make sure you have sourced ROS2 setup script as well as the local workspace setup script

```
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

Launch the node using the launch file provided

```
ros2 launch safety_node aeb
```
