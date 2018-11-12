#include <ros/ros.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/pubNewLine.h"


TEST(TESTSuite, testcase1)
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::pubNewLine>("pub_New_Line");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "service_rostest");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
