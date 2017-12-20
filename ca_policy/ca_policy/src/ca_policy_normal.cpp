
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
#include "ca_policy/ca_policy_normal.h"

namespace intelligent_ca
{
NormalCaPolicy::NormalCaPolicy(const ros::NodeHandle& ros_node) : CaPolicy(ros_node)
{
}

NormalCaPolicy::~NormalCaPolicy()
{
}

void NormalCaPolicy::executeReconfig()
{
  system("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS /opt/ca_policy/param/normal.yaml&");
}

void NormalCaPolicy::execute()
{
  ROS_INFO("executing Normal CA policy ...");

  executeReconfig();

  executeLED(CA_LED_ON);
}

}  // namespace
