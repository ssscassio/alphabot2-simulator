# Alphabot2 Ros Package and Simulator

## Installation and Dependencies

- ROS distro: [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Ubuntu version: Ubuntu 16.04 LTS (Xenial)

- Gazebo version: Gazebo 7.0

Perform the full installation for the ROS Kinetic that comes with Gazebo 7.0

## Building

Clone this repository into the src folder inside the catkin workspace and compile it.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ssscassio/alphabot2-simulator.git
cd ..
catkin_make
source devel/setup.bash
chmod +x src/alphabot2-simulator/**/*.py
```

## Running

### Simulation:

To launch the simulation world run:

```
roslaunch alphabot2_world spawn_world.launch
```

To spawn the robot inside the world run:

```
roslaunch alphabot2_world spawn_robot.launch
```

To launch the control node run:

```
roslaunch alphabot2_control alphabot2_control_gazebo.launch
```

To launch the pan tilt control node run:

```
roslaunch alphabot2_pantilt_control alphabot2_pantilt_control_gazebo.launch
```

### Real Robot:

To launch the control node run:

```
roslaunch alphabot2_control alphabot2_control_real.launch
```

To launch the pan tilt control node run:

```
roslaunch alphabot2_pantilt_control alphabot2_pantilt_control_real.launch
```

## Node Information

Topics:

- `/alphabot2_control`: Used to control the robot movement, `geometry_msgs/Twist` to be publish
- `/alphabot2_vertical`: Used to control **Tilt** from Pan-Tilt using `std_msgs/Float64` (degree between -90 and 90)
- `/alphabot2_horizontal`: Used to control **Pan** from Pan-Tilt using `std_msgs/Float64` (degree between -90 and 90)
- `/alphabot2_camera/image_raw`: Publishes `sensor_msgs/Image` from the camera module.

## Testing controls

To control robot movement publish a `geometry_msgs/Twist` to `/alphabot2_control` topic:

```
rostopic pub /alphabot2_control geometry_msgs/Twist (Press Tab)
```

To control camera movement publish a `std_msgs/Float64` to `/alphabot_horizontal` or `/alphabot_vertical` topics with a angle between -90 and 90 degrees:

**Pan**

```
rostopic pub /alphabot2_horizontal std_msgs/Float64 "data: 45"
```

**Tilt**

```
rostopic pub /alphabot2_vertical std_msgs/Float64 "data: -23"
```
