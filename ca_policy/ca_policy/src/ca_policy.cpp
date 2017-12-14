
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
#include <boost/thread/thread.hpp>
#include <vector>

#include <math.h>
#include <ros/ros.h>
//#include <ca_policy_msgs/People.h>
#include "ca_policy/ca_policy.h"
#include "object_bridge_msgs/ObjectMerged.h"
#include <tf/transform_listener.h>

namespace intelligent_ca
{
CaPolicy::CaPolicy() : name_(""), config_file_("")
{
}

CaPolicy::CaPolicy(const std::string& name, const std::string& config) : name_(name), config_file_(config)
{
}

CaPolicy::~CaPolicy()
{
}

bool CaPolicy::setConfiguration(const std::string& config)
{
  config_file_ = config;

  return true;
}

std::string CaPolicy::getConfiguration(void)
{
  return config_file_;
}

bool CaPolicy::setPolicyName(const std::string& name)
{
  name_ = name;

  return true;
}

std::string CaPolicy::getPolicyName(void)
{
  return name_;
}

}  // namespace
