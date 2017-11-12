/**
 *  MIT License
 *
 *  Copyright (c) 2017 Karan Vivek Bhargava
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    talkerTest.cpp
 *  @author  Karan Vivek Bhargava
 *  @copyright MIT License
 *
 *  @brief Assignment to implement ROS nodes
 *
 *  @section DESCRIPTION
 *
 *  This program is a part of the beginner tutorials in ROS
 *  It tests the talker node
 *
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_text.h"

/**
 * @brief      Tests whether the service exists
 *
 * @param[in]  TESTSuite          gtest framework
 * @param[in]  testServiceExists  Name of the test
 */
TEST(TESTSuite, testServiceExists) {
  // Create node handle
  ros::NodeHandle n;
  // Register the client to the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::change_text>("change_text");
  // Check if the service exist
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/**
 * @brief      Tests whether the service changes the text
 *
 * @param[in]  TESTSuite              gtest framework
 * @param[in]  testChangeTestService  Name of the test
 */
TEST(TESTSuite, testChangeTestService) {
  // Create a node handle
  ros::NodeHandle n;
  // Register the client to the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::change_text>("change_text");
  beginner_tutorials::change_text srv;
  // Set the input text
  srv.request.textip = "input";
  // Call the service
  client.call(srv);
  // Check the response
  EXPECT_STREQ("input", srv.response.textop.c_str());
}
