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
    : nh_("~"), listener_(ros::Duration(10)), 
    last_sonar_data_(INVALID_SONAR_DATA), last_sonar_data_deprecated_(-1), count_index(0)
  {
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::Range>("/water_uavcan_master/sonar_filtered", 1);
    scan_sub_ = nh_.subscribe("/water_uavcan_master/ultrasonic", 1000, &SonarFilter::update, this);

    nh_.param<std::string>("base_frame", base_frame_, "/base_link");

    sonar_dist_tolerance_ = 0.3; //TODO:control it in parameter
  }

  void SonarFilter::update(const sensor_msgs::Range& input_scan)
  {

   sensor_msgs::Range output_scan = input_scan;

   if (checkCorrectSonarDataAndSet(output_scan)) {
      publishSonarData(output_scan);
    } else {
      ROS_WARN("Abnormal Sonar data, do nothing but keep monitoring!");
    }

    /**
     * FIXME: Don't do anything for obstacle possiblity prediction, even if for redundent logging.
    if (getPossibilityOfObstacle() >= POSSIBILITY_MED ){
      ROS_WARN("Probably meet an obstacle, keep monitoring!");
    }*/
  }

  void SonarFilter::publishSonarData(const sensor_msgs::Range& pub)
  {
    scan_filtered_pub_.publish(pub);
  }
  
  bool SonarFilter::checkCorrectSonarDataAndSet(sensor_msgs::Range& data)
  {
    float sum=0.0;

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

      for(int i = 0; i < FILTER_NUMBER; i++) {
        sum += distance_array[i];
      }
      data.range = sum / FILTER_NUMBER;
      return true;
    }
  }
  
  void SonarFilter::setPossibilityOfObstacle(float range)
  {
    range < OBSTACLE_DISTANCE_THRESHOLD? possibility_obstacle_ = POSSIBILITY_HIGH :
      (range < last_sonar_data_? possibility_obstacle_ = POSSIBILITY_MED : possibility_obstacle_ = 
      POSSIBILITY_LOW);
  }
  ObstaclePosibility SonarFilter::getPossibilityOfObstacle()
  {
    return possibility_obstacle_;
  }
} //namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_filter_node");

  intelligent_ca::SonarFilter filter;
  ros::spin();
}


