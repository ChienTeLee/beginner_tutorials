## Overview
This repository is a simple ROS package created following ROS wiki tutorials. This project walks through:
- create pulisher node talker and subscriber node listener
- create one service in talker node to change publishing line
- use five logging levels of severity
- create a launch file to modify publish frequency
- create transformbroadcaster in talker node to broadcast tf frame
- create gtest/rostest to test service
- extend launch file to enable rosbag recording

The talker node keeps publishing a custom string message on a topic.
The listener node keeps printing out the received message through the topic.
The service in talker can change the publishing content.
The launchfile can change the publishing frequency.
The transformbroadcaster in talker keeps broadcasting its talker frame respect to world.
The launchfile is extended to use extra argument to enable rosbag recording

ROS wiki tutorials: http://wiki.ros.org/ROS/Tutorials

## Licence
- BSD 3-clause Liscense
```
Copyright <2018> <Chien-Te Lee>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions 
are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in 
the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from 
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
IN THE SOFTWARE.
```

## Dependencies
- Ubuntu 16.04
- CMake
- ROS kinetic
- gtest

## Standard install using command-line
- create catkin workspace
- clone beginner tutorial package into catkin workspace
- build catkin workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/ChienTeLee/beginner_tutorials.git
$ cd beginner_tutorials
$ git checkout Week11_HW
$ cd ~/catkin_ws
$ catkin_make
```

## How to run program using command line
- run ros master on first terminal
```
$ roscore
```

- run publisher on second terminal to publish at frequency of 5.5 Hz
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker 5.5
```

- run subscriber on third terminal
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```

- run service on fourth terminal to change publishing content
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosservice call /pub_New_Line "It is sunny now."
```


## How to run program using launch file
- run launch file on first terminal to publish at frequncy of 5.5 Hz
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch beginner_tutorials beginner_tutorial.launch pubFreq:=5.5
```

- run service on second terminal to change publishing content
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosservice call /pub_New_Line "It is sunny now."
```

## How to check tf frame
- run ros master on first terminal
```
$ roscore
```

- run publisher on second terminal to publish at frequency of 5.5 Hz
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker 5.5
```

- run tf_echo on third terminal to check tf frame
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun tf tf_echo
```

## How to run gtest
```
$ cd ~/catkin_ws
$ catkin_make run_tests
```

## How to enable rosbag in launchfile
- run launch file on first terminal to publish at frequncy of 5.5 Hz and enabel rosbag
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch beginner_tutorials beginner_tutorial.launch pubFreq:=5.5 record:=true
```

## How to replay rosbag record
- run ros master on first terminal
```
$ roscore
```

- run subscriber on second terminal
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```

- play rosbag on third terminal
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ cd ~/catkin_ws/src/beginner_tutorials/result
$ rosbag play talker_record.bag
```


