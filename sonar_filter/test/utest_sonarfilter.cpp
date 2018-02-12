/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

class UnitTestHelper
{
public:
  void cb(const sensor_msgs::Range& input_scan)
  {
    data = input_scan;
  }
  sensor_msgs::Range data;
};

TEST(UnitTestSonarFilter, ultra_m_filter)
{
  ros::NodeHandle nh("~");
  std::string ultra_m, ultrafilter_m;
  uint32_t test_retries_ = 5;

  nh.getParam("ultra_m", ultra_m);
  nh.getParam("ultrafilter_m", ultrafilter_m);
  UnitTestHelper h1;
  UnitTestHelper h2;

  ros::Subscriber sub1 = nh.subscribe(ultra_m, 10, &UnitTestHelper::cb, &h1);
  ros::Subscriber sub2 = nh.subscribe(ultrafilter_m, 10, &UnitTestHelper::cb, &h2);

  ros::WallDuration(2.0).sleep();

  while (test_retries_)
  {
    ros::spinOnce();

    ASSERT_EQ(sub1.getNumPublishers(), 1U);
    ASSERT_EQ(sub2.getNumPublishers(), 1U);

    EXPECT_NEAR(h1.data.range, h2.data.range, 0.01);

    test_retries_--;
  }

  EXPECT_EQ(test_retries_, 0);
}

TEST(UnitTestSonarFilter, ultra_l_filter)
{
  ros::NodeHandle nh("~");
  std::string ultra_l, ultrafilter_l;
  uint32_t test_retries_ = 5;

  nh.getParam("ultra_l", ultra_l);
  nh.getParam("ultrafilter_l", ultrafilter_l);
  UnitTestHelper h1;
  UnitTestHelper h2;

  ros::Subscriber sub1 = nh.subscribe(ultra_l, 10, &UnitTestHelper::cb, &h1);
  ros::Subscriber sub2 = nh.subscribe(ultrafilter_l, 10, &UnitTestHelper::cb, &h2);

  ros::WallDuration(2.0).sleep();

  while (test_retries_)
  {
    ros::spinOnce();

    ASSERT_EQ(sub1.getNumPublishers(), 1U);
    ASSERT_EQ(sub2.getNumPublishers(), 1U);

    EXPECT_EQ(h1.data.range, h2.data.range);

    test_retries_--;
  }

  EXPECT_EQ(test_retries_, 0);
}

TEST(UnitTestSonarFilter, ultra_r_filter)
{
  ros::NodeHandle nh("~");
  std::string ultra_r, ultrafilter_r;
  uint32_t test_retries_ = 5;

  nh.getParam("ultra_r", ultra_r);
  nh.getParam("ultrafilter_r", ultrafilter_r);
  UnitTestHelper h1;
  UnitTestHelper h2;

  ros::Subscriber sub1 = nh.subscribe(ultra_r, 10, &UnitTestHelper::cb, &h1);
  ros::Subscriber sub2 = nh.subscribe(ultrafilter_r, 10, &UnitTestHelper::cb, &h2);

  ros::WallDuration(2.0).sleep();

  while (test_retries_)
  {
    ros::spinOnce();

    ASSERT_EQ(sub1.getNumPublishers(), 1U);
    ASSERT_EQ(sub2.getNumPublishers(), 1U);

    EXPECT_NEAR(h1.data.range, h2.data.range, 0.01);

    test_retries_--;
  }

  EXPECT_EQ(test_retries_, 0);
}

TEST(UnitTestSonarFilter, ultraOutOfBound)
{
  ros::NodeHandle nh("~");
  std::string ultra_l, ultrafilter_l;
  uint32_t test_retries_ = 5;

  nh.getParam("ultra_l", ultra_l);
  nh.getParam("ultrafilter_l", ultrafilter_l);
  UnitTestHelper h1;
  UnitTestHelper h2;

  ros::Subscriber sub1 = nh.subscribe(ultra_l, 10, &UnitTestHelper::cb, &h1);
  ros::Subscriber sub2 = nh.subscribe(ultrafilter_l, 10, &UnitTestHelper::cb, &h2);

  ros::WallDuration(2.0).sleep();

  while (test_retries_)
  {
    ros::spinOnce();

    ASSERT_EQ(sub1.getNumPublishers(), 1U);
    ASSERT_EQ(sub2.getNumPublishers(), 1U);

    EXPECT_EQ(h1.data.max_range, 1.0);
    EXPECT_EQ(h1.data.min_range, 1.0);

    test_retries_--;
  }

  EXPECT_EQ(test_retries_, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utest_sonarfilter");
  return RUN_ALL_TESTS();
}
