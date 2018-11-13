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

/** @file test.cpp
 *  @brief Implementation of simple rostest
 *  @copyright (c) 2018 Chien-Te Lee
 *  @author Chien-Te Lee
 *  @date   11/12/2018
 *
 *  This program implemnts testcase to test talker node service "pub_New_Line".
 *  
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/pubNewLine.h"

/**
 *  @brief This is a simple testcase to test talker node service "pub_New_Line"
 *  @param TESTSuite is the name of the test suite
 *  @param testcase1 is the name of the testcase
 *  @return none
 */
TEST(TESTSuite, testcase1)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::pubNewLine>("pub_New_Line");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}

/**
 *  @brief main fucntion to run all testcases
 *  @param argc is the number of input argument
 *  @param argv are the input arguments
 *  @return 0 if all the tests are successful
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "service_rostest");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


