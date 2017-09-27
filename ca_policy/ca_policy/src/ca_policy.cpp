
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

namespace intelligent_ca {


  CaPolicy::CaPolicy()
  :name_(""), config_file_("")
  {

  }

  CaPolicy::CaPolicy(const std::string& name, const std::string& config)
  :name_(name), config_file_(config)
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
    config_file_ = name;
    
    return true;
  }
  
  std::string CaPolicy::getPolicyName(void)
  {
    return name_;
  }

} //namespace


