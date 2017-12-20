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
#ifndef ICA_SOCIAL_CA_POLICY_H
#define ICA_SOCIAL_CA_POLICY_H

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include "ca_policy/ca_policy.h"

namespace intelligent_ca
{
/** @brief Social CA Policy class */
class SocialCaPolicy : public CaPolicy
{
public:
  SocialCaPolicy(const ros::NodeHandle& ros_node);
  // SocialCaPolicy(ros::NodeHandle* ros_node, const std::string& name, const std::string& config);
  virtual ~SocialCaPolicy();

  /** @brief execute the policy*/
  void execute();

private:
  /** @brief implementation for dynamic reconfiguration to navigation stack. */
  void executeReconfig();
};

}  // namespace
#endif
