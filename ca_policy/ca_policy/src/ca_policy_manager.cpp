/******************************************************************************
 *
 Copyright (c) 2017, Intel Corporation *
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

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
  try{
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
  catch (std::bad_alloc& ba){
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

    if (name == "normal") {
     // system("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS /opt/ca_policy/param/normal.yaml");

    }else if (name == "social") {
     // system("rosrun dynamic_reconfigure dynparam load /move_base/DWAPlannerROS /opt/ca_policy/param/social.yaml");
    }

    return true;
  }
  else
  {
    ROS_WARN("No policy named %s", name.c_str());
    return false;
  }

}

} // namespace
