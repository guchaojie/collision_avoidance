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

#ifndef SONAR_FILTER_SONAR_FILTER_H
#define SONAR_FILTER_SONAR_FILTER_H

namespace intelligent_ca
{
const int FILTER_NUMBER = 10;

class SonarFilter
{
public:
  SonarFilter();
  void update1(const sensor_msgs::Range& input_scan);
  void update2(const sensor_msgs::Range& input_scan);
  void update3(const sensor_msgs::Range& input_scan);

private:
  bool checkCorrectSonarDataAndSet(sensor_msgs::Range& data, std::vector<float>& distance_array, int& count_index);
  void publishSonarData(const sensor_msgs::Range& pub);
  void handleSonarData(sensor_msgs::Range& data, std::vector<float>& distance_array, int& count_index);
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string ultrasonic_l1_frame_;
  std::string ultrasonic_m_frame_;
  std::string ultrasonic_r1_frame_;
  std::string base_frame_;
  std::vector<ros::Publisher> scan_filtered_pub_;
  std::vector<ros::Subscriber> scan_sub_;
  XmlRpc::XmlRpcValue filter_list;
  std::vector<float> distance_array1;
  int count_index1;
  std::vector<float> distance_array2;
  int count_index2;
  std::vector<float> distance_array3;
  int count_index3;
};

}  // namespace intelligent_ca
#endif  // SONAR_FILTER_SONAR_FILTER_H
