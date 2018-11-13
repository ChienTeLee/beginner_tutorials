/**
 * BSD 3 clauses Liscense
 *
 * Copyright <2018> <Chien-Te Lee>
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from 
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file talker.cpp
 *  @brief Implementation of talker node
 *  @copyright (c) 2018 Chien-Te Lee
 *  @author Chien-Te Lee
 *  @date   11/6/2018
 *
 *  This program implemnts talker node to publish content, service to change the publishing content, and TransformBroadcaster to
 *  broadcast TF frame.
 *  
 */


#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include "beginner_tutorials/pubNewLine.h"

std::string line = "It is rainy today.";

/**
 *  @brief This is a fucntion to change publishing content
 *  @param req is the string of input argument
 *  @param res is the output response
 *  @return true if the publishing content correctly changed
 */
bool pubNewLine(beginner_tutorials::pubNewLine::Request &req,
                beginner_tutorials::pubNewLine::Response &res) {
  res.outLine = req.inLine;
  line = res.outLine;
  ROS_WARN_STREAM("publishing new line");
  return true;
}


/**
 *  @brief the pipeline of creating talker node, change line service, and Tf broadcaster
 *  @param argc is the number of input argument
 *  @param argv are the input arguments
 *  @return if the progran executes successfully
 */

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  // creater chatter
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  if (chatter_pub) {
    ROS_DEBUG_STREAM("chatter publishing correctly");
  } else {
    ROS_FATAL_STREAM("chatter not publishing correctly");
    return 1;
  }

  // create service
  ros::ServiceServer service = n.advertiseService("pub_New_Line", pubNewLine);
  if (service) {
    ROS_DEBUG_STREAM("service working correctly");
  } else {
    ROS_FATAL_STREAM("service not working correctly");
    return 1;
  }

  // check if publish frequency is correct
  double pubFreq = 10.0;
  if (argc == 2) {
    pubFreq = std::stod(argv[1]);
  } else if (argc > 2) {
    ROS_WARN_STREAM("too much argument number, change freq to 10 Hz");
    pubFreq = 10.0;
  }

  if (pubFreq <= 0.0) {
    ROS_ERROR_STREAM("freq <= 0 invalid, change freq to 10 Hz");
    ROS_ERROR_STREAM("negative freq is " << pubFreq << " Hz");
    pubFreq = 10.0;
  }
  ROS_INFO_STREAM("publish at rate " << pubFreq << " Hz");
  ros::Rate loop_rate(pubFreq);


  // create Tf transform and do traslation and rotation
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(7.0, 5.0, 10.0));
  tf::Quaternion q;
  q.setRPY(M_PI/6, M_PI/3, M_PI/2);
  transform.setRotation(q);


  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << line << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    ROS_DEBUG_STREAM("currently publish at rate " << pubFreq << " Hz");

    chatter_pub.publish(msg);

    // broadcast Tf frame
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                     "world", "talk"));


    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
