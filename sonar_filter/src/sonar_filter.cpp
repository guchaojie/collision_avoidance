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

#include <sonar_filter/sonar_filter.h>

namespace intelligent_ca {


  SonarFilter::SonarFilter()
    : nh_("~"), listener_(ros::Duration(10)), count_index1(0), count_index2(0), count_index3(0)
  {
    if (nh_.hasParam("SonarFilters"))
    {
      nh_.getParam("SonarFilters", filter_list);
      for (int32_t i = 0; i < filter_list.size(); ++i)
      {
        std::string input_topic = static_cast<std::string>(filter_list[i]["subscribe"]);
        std::string output_topic = static_cast<std::string>(filter_list[i]["publish"]);

        switch (i) {
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

  void SonarFilter::handleSonarData(sensor_msgs::Range& data,
          std::vector<float>& distance_array, int& count_index)
  {

    if (checkCorrectSonarDataAndSet(data, distance_array, count_index)) {
      publishSonarData(data);
    } else {
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
      if (found != std::string::npos) {
   //   ROS_INFO("SonarFilter: published to topic %s", scan_filtered_pub_[i].getTopic().c_str());
        scan_filtered_pub_[i].publish(pub);
        break;
      }
    }
  }

  bool SonarFilter::checkCorrectSonarDataAndSet(sensor_msgs::Range& data,
          std::vector<float>& distance_array, int& count_index)
  {

    if (count_index < FILTER_NUMBER) {
      distance_array.push_back(data.range);
      count_index++;
      return false;
    }
    else {
      for(int i = 0;i < (FILTER_NUMBER-1); i++) {
      distance_array[i]=distance_array[i+1];
      }

      distance_array[FILTER_NUMBER-1] = data.range;
      if (data.range > data.max_range)
         return true;

      float sum=0.0;
      for(int i = 0; i < FILTER_NUMBER; i++) {
        sum += distance_array[i];
      }
      data.range = (float)(sum / FILTER_NUMBER);
      return true;
    }
  }

} //namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_filter_node");

  intelligent_ca::SonarFilter filter;
  ros::MultiThreadedSpinner s(3);
  ros::spin(s);

  return 0;
}


