## Overview
This repository is a simple ROS package created following ROS wiki tutorials. This project walks through:
- creating and building a beginner tutorial package
- writing a simple publisher (talker)
- writing a simple subscriber (subscriber)

The publisher node keep publishing a custom string message on a topic. The subscriber keeps printing out the received message throgh the topic.

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

## Standard install via command-line
- create catkin workspace
- clone package into catkin workspace
- build catkin workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/ChienTeLee/beginner_tutorials.git
$ cd ~/catkin_ws
$ catkin_make
```

## How to run program
- run ros master on first terminal
```
$ roscore
```

- run publisher on second terminal
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials talker
```

- run subscriber on third terminal
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rosrun beginner_tutorials listener
```





