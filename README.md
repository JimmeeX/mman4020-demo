# ROS | Ardupilot + SITL | Arduino Interface

For the demo (simulation + control system to interface with prototype).

## Project Structure

- /Arduino - Code to Program the Arduino
- /launch - Launch Files

Feel free to add ROS packages if need to.

## Installation

Tested on Ubuntu 18.04 LTS

1. [Install SITL & Mavproxy](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#sitl-simulator-software-in-the-loop)

2. [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)

3. [Install MAVROS (Link SITL with ROS)](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation) (be mindful of ros version)

4. [Install Arduino IDE, to program the Arduino](https://www.arduino.cc/en/main/software)

5. [Install rosserial (Link Arduino with ROS)](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) (be mindful of ros version)

6. Clone this Repo

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

- Get ROS to listen to SITL
- Get ROS to listen to Arduino
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
