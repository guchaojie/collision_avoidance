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

  enum ObstaclePosibility {
    POSSIBILITY_LOW,
    POSSIBILITY_MED,
    POSSIBILITY_HIGH
  };

  const float INVALID_SONAR_DATA = -1000.0;

  ///@brief distance threshold of the distance. Unit:meter.
  const float OBSTACLE_DISTANCE_THRESHOLD = 0.25;

class SonarFilter
{
public:
  SonarFilter();
  void update(const sensor_msgs::Range& input_scan);

private:
  bool checkCorrectSonarDataAndSet(sensor_msgs::Range& data);
  void publishSonarData(const sensor_msgs::Range& pub);

  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  std::string base_frame_;
  ros::Publisher scan_filtered_pub_;
  ros::Subscriber scan_sub_;

  /**
   * @brief The last correct sonar data
   */
  float last_sonar_data_;

  /**
   * @brief The last deprecated sonar data, which is far away (>sonar_dist_tolerance_) from the
   l ast correct data.      *
   */
  float last_sonar_data_deprecated_;

  float sonar_dist_tolerance_;
  std::vector<float> distance_array;
  int count_index;
  ObstaclePosibility possibility_obstacle_;
};


} //namespace
#endif
