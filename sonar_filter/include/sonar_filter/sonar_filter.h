/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017-2018, Intel, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_listener.h>

#ifndef ICA_SONAR_FILTER_H_
#define ICA_SONAR_FILTER_H_

namespace intelligent_ca {

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


} //namespace
#endif
