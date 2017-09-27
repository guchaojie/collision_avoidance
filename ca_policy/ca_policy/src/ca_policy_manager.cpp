
/******************************************************************************  *
Copyright (c) 2017, Intel Corporation                                           *
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

#include <vector>
#include <boost/thread/thread.hpp>

#include <math.h>
#include <ros/ros.h>
//#include <ca_policy_msgs/People.h>
#include <tf/transform_listener.h>
#include <object_bridge_msgs/ObjectMerged.h>
#include <ca_policy/ca_policy.h>
#include <ca_policy/consts.h>
#include <ca_policy/ca_policy_manager.h>

namespace intelligent_ca {


  CaPolicyManager::CaPolicyManager()
  {
    policies_.clear();
    current_policy_ = nullptr;
  }

  CaPolicyManager::~CaPolicyManager()
  {
  }
  
  bool CaPolicyManager::addPolicy(const std::string name, const CaPolicy& policy)
  {
    try{
      CaPolicyVector::iterator found = findPolicy(name);
      if(found){
        std::get<1>(*found) = policy;
        return true;
      }
      
      CaPolicyPair pair = std::make_pair(name, policy);
      policies_.push_back(pair);
      return true;
    }catch (...){
      ROS_ERROR("Error when add policy...");
      return false;
    }
  }

  bool CaPolicyManager::deletePolicy(const std::string name)
  {
    try{
      CaPolicyVector::iterator found = findPolicy(name);
      if (found){
        policies_.erase(*found);    
      }
      return true;
    }catch (...){
      ROS_ERROR("Error when add policy...");
      return false;
    }
  }
  
  CaPolicyVector::iterator CaPolicyManager::findPolicy(const std::string name)
  {
    for (CaPolicyVector::interator it = policies_.begin(); it != policies_.end(); ++it){
      if (name == std::get<0> (*it)){
        return it;
      }
    }
    
    return nullptr;
  }
  
   
  bool CaPolicyManager::setCurrentPolicy(const std::string name)
  {
    try{
      CaPolicyVector::iterator found = findPolicy(name);
      if(found != nullptr)
      current_policy_ = found;
      
      
      ///TODO: talk with Navigation stack here
      
      
      return true;
    }catch(...){
      ROS_ERROR("Error when set current Policy");
      return false;
    }
   
  }

} //namespace


