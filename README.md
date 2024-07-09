# tiago_moveit

ROS 2 package for moving [Pal Robotics Tiago robot](https://github.com/pal-robotics/tiago_robot) arm using MoveIt Motion Planning Framework.

## Download tiago_simulation

```shell
$ cd ~/ros2_ws/src
$ git clone -b humble-devel https://github.com/pal-robotics/tiago_simulation.git
```

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone git@github.com:agutf/tiago_moveit.git
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
```

## Usage

### With Gazebo

```shell
$ ros2 launch tiago_gazebo tiago_gazebo.launch.py moveit:=True is_public_sim:=True
```

```shell
$ ros2 launch move_tiago move_tiago.launch.py use_sim_time:=True
```

### With Tiago

```shell
$ ros2 launch tiago_moveit_config move_group.launch.py
```

```shell
$ ros2 launch move_tiago move_tiago.launch.py use_sim_time:=False
```

