
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
#include <ros/ros.h>
#include "ca_policy/ca_policy.h"
#include "object_bridge_msgs/ObjectMerged.h"
#include <tf/transform_listener.h>

namespace intelligent_ca
{
CaPolicy::CaPolicy(const ros::NodeHandle& ros_node) : nh_(ros_node), name_(""), config_file_("")
{
  nh_.param("luminance", luminance_, 128);
  ca_led_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/water_uavcan_master/set_led", 100);
}

CaPolicy::~CaPolicy()
{
}

bool CaPolicy::setConfiguration(const std::string& config)
{
  config_file_ = config;

  return true;
}

bool CaPolicy::setPolicyName(const std::string& name)
{
  name_ = name;

  return true;
}

void CaPolicy::executeLED(LEDState state)
{
  led_array_.data.clear();
  led_array_.data.resize(4, 0);

  /**< set LED state. */
  led_array_.data[0] = state;

  /**< set LED luminance. */
  led_array_.data[3] = luminance_;

  ca_led_pub_.publish(led_array_);
}

}  // namespace
