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

//#include <math.h>
#include <ros/ros.h>
//#include <ca_policy_msgs/People.h>
#include "ca_policy/ca_policy.h"
#include "ca_policy/ca_policy_manager.h"
#include "ca_policy/consts.h"
#include "object_bridge_msgs/ObjectMerged.h"
#include <tf/transform_listener.h>

namespace intelligent_ca
{
CaPolicyManager::CaPolicyManager()
{
  policies_.clear();
}

CaPolicyManager::~CaPolicyManager()
{
  policies_.clear();
}

bool CaPolicyManager::addPolicy(const std::string name, const std::shared_ptr<CaPolicy>& policy)
{
  try
  {
    CaPolicyVector::iterator exist;
    bool found = findPolicy(name, exist);
    if (found)
    {
      ROS_INFO("Policy %s already exist, update it with new content.", name.c_str());
      std::get<1>(*exist) = policy;
      return true;
    }

    ROS_INFO("Create new policy %s ...", name.c_str());
    CaPolicyPair pair = std::make_pair(name, policy);
    policies_.push_back(pair);
    return true;
  }
  catch (std::bad_alloc& ba)
  {
    // std::cerr << "bad_alloc caught: " << ba.what() << '\n';
    ROS_INFO("bad_alloc caught: %s", ba.what());
    return false;
  }
  catch (...)
  {
    ROS_INFO("Error when add policy...");
    return false;
  }
}

bool CaPolicyManager::deletePolicy(const std::string name)
{
  CaPolicyVector::iterator exist;
  bool found = findPolicy(name, exist);
  if (found)
  {
    policies_.erase(exist);
    return true;
  }

  return false;
}

bool CaPolicyManager::isPolicyExist(const std::string name)
{
  for (CaPolicyVector::iterator it = policies_.begin(); it != policies_.end(); ++it)
  {
    if (name == std::get<0>(*it))
    {
      return true;
    }
  }

  return false;
}

bool CaPolicyManager::findPolicy(const std::string name, CaPolicyVector::iterator& pair)
{
  for (CaPolicyVector::iterator it = policies_.begin(); it != policies_.end(); ++it)
  {
    if (name == std::get<0>(*it))
    {
      pair = it;
      return true;
    }
  }

  return false;
}

std::string CaPolicyManager::getCurrentPolicy()
{
  return std::get<0>(current_policy_);
}
bool CaPolicyManager::setCurrentPolicy(const std::string name)
{
  CaPolicyVector::iterator exist;
  bool found = findPolicy(name, exist);
  if (found)
  {
    current_policy_ = *exist;

    if (name == "normal")
    {
      ROS_INFO("Setting the normal configuraiton to Robot");
      system("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS /opt/ca_policy/param/normal.yaml&");
    }
    else if (name == "social")
    {
      ROS_INFO("Setting the social configuraiton to Robot");
      system("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS /opt/ca_policy/param/social.yaml&");
    }

    return true;
  }
  else
  {
    ROS_WARN("No policy named %s", name.c_str());
    return false;
  }
}

}  // namespace
