# ROS | Ardupilot + SITL | Arduino Interface

For the demo (simulation + control system to interface with prototype).

## Project Structure

- /src/arduino - Code to Program the Arduino
- /src/ros/gui - Code for the Web GUI
- /src/ros/sampler - Main Programming Logic/Algorithms
- /scripts - Any scripts to automate stuff (I haven't written any lel)

Feel free to add ROS packages if need to.

## Installation

Tested on Ubuntu 18.04 LTS

1. [Install SITL & Mavproxy](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-simulator-software-in-the-loop)

2. [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)

3. [Install MAVROS (Link SITL with ROS)](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation) (be mindful of ros version)

4. [Install Arduino IDE, to program the Arduino](https://www.arduino.cc/en/main/software)

5. [Install rosserial (Link Arduino with ROS)](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) (be mindful of ros version)

6. [Install rosbridge-server (Link ROS to Web GUI)](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality)

```shell
sudo apt-get install ros-melodic-rosbridge-server
```

7. Follow [this](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to setup your ROS environment. Essentially create a ~/catkin_ws folder, then clone this repo & copy its contents into ~/catkin_ws/src.

8. Build ROS Packages

```shell
cd ~/catkin_ws && catkin_make
source source ~/catkin_ws/devel/setup.bash
```

## Usage

1. Start SITL

| Param | Name                                                         |
| ----- | ------------------------------------------------------------ |
| -v    | Vehicle                                                      |
| -L    | Location (defined in ardupilot/Tools/autotest/locations.txt) |

```shell
sim_vehicle.py -v ArduCopter -L Delta
```

2. Run ROS Launch File

This will

- Get ROS to interface to SITL Simulation prgoram
- Get ROS to interface with Arduino program
- Get ROS to interface with Web GUI
- Run ROS Main Code

Format

```shell
roslaunch sampler main.launch port:=<arduino port>
```

Example if Arduino is connected to computer's /dev/ttyACM0 port

```shell
roslaunch sampler main.launch port:=/dev/ttyACM0
```

NOTE: I might write a script to automate this later on.

1. To see the GUI, launch a Web Server to open up /src/ros/gui/index.html. You can use the following

```
python -m SimleHTTPServer <port> <filename>
```

alternatively, if you use VSCode, you can "Go Live"
