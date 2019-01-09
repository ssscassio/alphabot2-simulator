# Alphabot2 Ros Package and Simulator

## Installation and Dependencies

- ROS distro: [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Ubuntu version: Ubuntu 16.04 LTS (Xenial)

- Gazebo version: Gazebo 7.0

Perform the full installation for the ROS Kinetic that comes with Gazebo 7.0

- _To real robot: Install [Raspcam_node](https://github.com/UbiquityRobotics/raspicam_node) following [the tutorial](https://github.com/UbiquityRobotics/raspicam_node#installation)_.

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

To launch the pan tilt control node run:

```
roslaunch alphabot2_pantilt alphabot2_pantilt_gazebo.launch
```

To start the bottom sensors middleman node run:

```
rosrun alphabot2_sensors_middleman BottomSensorsMiddleman.py
```

To start the top sensors middleman node run:

```
rosrun alphabot2_sensors_middleman TopSensorsMiddleman.py
```

To launch the robot movement created by the sensors node run:

```
roslaunch alphabot2_control gazebo_sensors_control.launch
```

### Real Robot:

To launch the control node run:

```
roslaunch alphabot2_control alphabot2_control_real.launch
```

To launch the pan tilt control node run:

```
roslaunch alphabot2_pantilt alphabot2_pantilt_control_real.launch
```

To start the alphabot2 node to handler with top sensors, bottom sensors and drive control run:

```
roslaunch alphabot2_control alphabot2_robot_handler.launch
```

To launch the robot movement created by the sensors node run:

```
roslaunch alphabot2_control alphabot2_sensors_control.launch
```

## ROS architecture

![Rosgraph for the simulation world](pictures/arch-gazebo.png)

<p align="center"> 
<img src="pictures/arch-real.png" align="left" height=365 alt="Rosgraph for real alphabot">
<img src="pictures/arch.png" align="right" height=365 alt="Rosgraph for sensors and control using sensors">
</p>
<br><br>

### Description of ROS nodes

- `alphabot2_control_node_real`: Subscribe to `/alphabot2/control` and translate it to drive control the Alphabot2. (_This node only runs on a RaspberryPI_)
- `alphabot2_pantilt_node_real`: Subscribe to `/alphabot2/vertical` and `/alphabot2/horizontal` topics and control the Pan and Tilt position of the RaspCam on Alphabot2. (_This node only runs on a RaspberryPI_)
- `alphabot2_pantilt_node_gazebo`: Same of `alphabot2_pantilt_node_real` but controls Gazebo model virtual joints instead of real robot Pan-Tilt.
- `alphabot2_top_sensors_middleman`: Receives info from Gazebo top sensors and retransmits in standardized format to `/alphabot2/top_sensors`.
- `alphabot2_bottom_sensors_middleman`: Receives info from Gazebo bottom sensors and retransmits in standardized format to `/alphabot2/bottom_sensors`.
- `alphabot2_handler`: Read information provide by Alphabot2's Infrared sensors and retransmits in standardized format to `/alphabot2/top_sensors` and `/alphabot2/bottom_sensors`. Also subscribe to `/alphabot2/control` and translate it to drive control the Alphabot2. (_This node only runs on a RaspberryPI_)

### Description of ROS Topics:

- `/alphabot2/control`: Used to control the robot movement, `geometry_msgs/Twist` to be publish
- `/alphabot2/vertical`: Used to control **Tilt** from Pan-Tilt using `std_msgs/Float64` (degree between -90 and 90)
- `/alphabot2/horizontal`: Used to control **Pan** from Pan-Tilt using `std_msgs/Float64` (degree between -90 and 90)
- `/alphabot2/camera/image_raw`: Publishes `sensor_msgs/Image` from the camera module.
- `/alphabot2/top_sensors`: Receives the result of the top sensors to be used by the real robot and the simulated robot. (Message type: `std_msgs/Int32MultiArray`)
- `/alphabot2/bottom_sensors`: Receives the result of the bottom sensors to be used by the real robot and the simulated robot. (Message type: `std_msgs/Int32MultiArray`)

## Testing controls

To control robot movement publish a `geometry_msgs/Twist` to `/alphabot2/control` topic:

```
rostopic pub /alphabot2/control geometry_msgs/Twist (Press Tab)
```

To control camera movement publish a `std_msgs/Float64` to `/alphabot_horizontal` or `/alphabot2/vertical` topics with a angle between -90 and 90 degrees:

**Pan**

```
rostopic pub /alphabot2/horizontal std_msgs/Float64 "data: 45"
```

**Tilt**

```
rostopic pub /alphabot2/vertical std_msgs/Float64 "data: -23"
```

To check that it's being published the correct sensors results to the real robot and the simulated robot:

**Top Sensors**

```
rostopic echo /alphabot2/top_sensors
```

**Bottom Sensors**

```
rostopic echo /alphabot2/bottom_sensors
```

## Papers related

- [**Sensors Paper**](https://github.com/ssscassio/alphabot2-simulator/blob/master/Report/Line_Following_and_Obstacle_Avoidance_behaviours_for_AlphaBot2_RPi___real_and_simulated.pdf)
- [**Camera Paper**](https://github.com/ssscassio/alphabot2-simulator/blob/master/Report/Development_of_AlphaBot2_Gazebo_simulator_for_RPi_camera.pdf)
