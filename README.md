# DJI Robomaster S1 ROS Driver

This repository consist of a ROS wrapper writter on top of Robomaster SDK. The ROS packages under this repository aims to control of DJI's cheap omni wheel robot, Robomaster S1, via ROS.
## Installation

### Build from source

`$ cd <catkin_ws/src>`
`$ git clone https://github.com/erdemuysalx/robomaster_s1_ros.git`
`$ cd ..`
`$ catkin build`
`$ source devel/setup.bash`

## Structure

```mermaid
graph LR
A[Robomaster S1] -- Python SDK --> B((Talker Node))
B -- ROS Topic --> D{ROS Master}
A -- Python SDK --> C((Listener Node)) -- ROS Topic --> D
```on SDK --> C((Listener Node)) -- ROS Topic --> D
```

## Nodes

### Talker Node

This node reads/listens the data produced at Robomaster using Python SDK, and then writes/talks the obtained data to the relevant ROS topics as ROS messages.
#### Published Topics
`sensor_msgs/Image.msg` to  `/robomaster/image`

`sensor_msgs/Imu.msg` to `/robomaster/imu`

`sensor_msgs/Quaternion.msg` to `/robomaster/attitude`

`---` to `/robomaster/esc`

`std_msgs/Float64.msg` to `/robomaster/battery`

### Listener Node

This node reads/listens the ROS messages which are sent from other machines, such as a remote computer, to the relevant topics, and then processes the incoming messages with the Robomaster Python SDK, allowing the robot to take action.    

#### Subscribed Topics
`geometyr_msgs/Twist.msg` from `/robomaster/chassis`

`geometyr_msgs/Twist.msg` from `/robomaster/gimbal`

### Parameters
`~robomaster/ip`

`~robomaster/version`

`~robomaster/sn`

`~robomaster/chassis_mode`

`~robomaster/gimbal_mode`

`~robomaster/chassis/static_flag`

`~robomaster/chassis/up_hil`

`~robomaster/chassis/down_hill`

`~robomaster/chassis/on_slope`

`~robomaster/chassis/pick_up`

`~robomaster/chassis/impact_x`

`~robomaster/chassis/impact_y`

`~robomaster/chassis/impact_z`

`~robomaster/chassis/roll_over`

`~robomaster/chassis/hill_static`