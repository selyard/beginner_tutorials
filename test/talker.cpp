/**
*  @file talker.cpp
 * @brief ROS Broadcaster with additional service which modifies a string
 * provided by the user in the console when called.
 *
 * @copyright Copyright 2020, Spencer Elyard [MIT License]
 */

#include <sstream>
#include <string>
#include <exception>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include <ros/service_client.h>
#include "beginner_tutorials/modifyString.h"
#include "std_msgs/String.h"

#include <gtest/gtest.h>
#include "beginner_tutorials/modifyString.h"

std::shared_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, talker)
{
  ros::ServiceClient client = nh->serviceClient<beginner_tutorials::modifyString>("make_string_better");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::modifyString service;
  std::string test_string = "test string";
  service.request.inputString = test_string;
  client.call(service);

  EXPECT_EQ(service.response.outputString, test_string + ", but better!");
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"test_talker");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
