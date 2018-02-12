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
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>

#include <sonar_filter/sonar_filter.h>

namespace intelligent_ca
{
SonarFilter::SonarFilter() : nh_("~"), listener_(ros::Duration(10)), count_index1(0), count_index2(0), count_index3(0)
{
  if (nh_.hasParam("SonarFilters"))
  {
    nh_.getParam("SonarFilters", filter_list);
    for (int32_t i = 0; i < filter_list.size(); ++i)
    {
      std::string input_topic = static_cast<std::string>(filter_list[i]["subscribe"]);
      std::string output_topic = static_cast<std::string>(filter_list[i]["publish"]);

      switch (i)
      {
        case 0:
          scan_sub_.push_back(nh_.subscribe(input_topic, 1000, &SonarFilter::update1, this));
          break;
        case 1:
          scan_sub_.push_back(nh_.subscribe(input_topic, 1000, &SonarFilter::update2, this));
          break;
        case 2:
          scan_sub_.push_back(nh_.subscribe(input_topic, 1000, &SonarFilter::update3, this));
          break;
        default:
          ROS_WARN("Sonar Filter has no more channel to support data filtering");
      }

      ROS_INFO("SonarFilter: subscribed to topic %s", scan_sub_.back().getTopic().c_str());
      scan_filtered_pub_.push_back(nh_.advertise<sensor_msgs::Range>(output_topic, 1));
    }
  }

  nh_.param<std::string>("base_frame", base_frame_, "/base_link");
  nh_.param<std::string>("ultrasonic_l1_frame", ultrasonic_l1_frame_, "");
  nh_.param<std::string>("ultrasonic_m_frame", ultrasonic_m_frame_, "");
  nh_.param<std::string>("ultrasonic_r1_frame", ultrasonic_r1_frame_, "");
}

void SonarFilter::handleSonarData(sensor_msgs::Range& data, std::vector<float>& distance_array, int& count_index)
{
  if (checkCorrectSonarDataAndSet(data, distance_array, count_index))
  {
    publishSonarData(data);
  }
  else
  {
    ROS_WARN("Abnormal Sonar data, do nothing but keep monitoring!");
  }
}

void SonarFilter::update1(const sensor_msgs::Range& input_scan)
{
  sensor_msgs::Range output_scan = input_scan;

  handleSonarData(output_scan, distance_array1, count_index1);
}

void SonarFilter::update2(const sensor_msgs::Range& input_scan)
{
  sensor_msgs::Range output_scan = input_scan;

  handleSonarData(output_scan, distance_array2, count_index2);
}

void SonarFilter::update3(const sensor_msgs::Range& input_scan)
{
  sensor_msgs::Range output_scan = input_scan;

  handleSonarData(output_scan, distance_array3, count_index3);
}

void SonarFilter::publishSonarData(const sensor_msgs::Range& pub)
{
  for (int32_t i = 0; i < filter_list.size(); ++i)
  {
    std::string input_topic = static_cast<std::string>(filter_list[i]["subscribe"]);
    std::size_t found = input_topic.find(pub.header.frame_id);
    if (found != std::string::npos)
    {
      //   ROS_INFO("SonarFilter: published to topic %s", scan_filtered_pub_[i].getTopic().c_str());
      scan_filtered_pub_[i].publish(pub);
      break;
    }
  }
}

bool SonarFilter::checkCorrectSonarDataAndSet(sensor_msgs::Range& data, std::vector<float>& distance_array,
                                              int& count_index)
{
  if (count_index < FILTER_NUMBER)
  {
    distance_array.push_back(data.range);
    count_index++;
    return false;
  }
  else
  {
    for (int i = 0; i < (FILTER_NUMBER - 1); i++)
    {
      distance_array[i] = distance_array[i + 1];
    }

    distance_array[FILTER_NUMBER - 1] = data.range;
    if (data.range > data.max_range)
      return true;

    float sum = 0.0;
    for (int i = 0; i < FILTER_NUMBER; i++)
    {
      sum += distance_array[i];
    }
    data.range = static_cast<float>(sum / FILTER_NUMBER);
    return true;
  }
}

}  // namespace intelligent_ca

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_filter_node");

  intelligent_ca::SonarFilter filter;
  ros::MultiThreadedSpinner s(3);
  ros::spin(s);

  return 0;
}
