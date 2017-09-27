
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
#include <ca_policy_msgs/People.h>
#include <object_msg_merger_msgs/ObjectMerged.h> ///TODO: name to be defined


namespace intelligent_ca {


  ObstacleFromCamera::ObstacleFromCamera()
  {
    velocity_.x = 0.0f;
    velocity_.y = 0.0f;
    velocity_.z = 0.0f;

  }
  
  ObstacleFromCamera::ObstacleFromCamera(const objectMergedStamped& object)
  {
    try{
      id_ = object.id;
      type_ = object.type;
      frame_id_ = object.header.frame_id;
      stamp_ = object.header.stamp;
      pos_min_ = object.min;
      pos_max_ = object.max;
      
      roi_ = object.roi;
    }
    catch(...)
    {
      ROS_DEBUG("invalid parameter in construct func.");
    }
  }
  
  ObstacleFromCamera::ObstacleFromCamera(const objectMerged& object)
  {
    try{
      id_ = object.id;
      type_ = object.type;
      pos_min_ = object.min;
      pos_max_ = object.max;
    }
    catch(...)
    {
      ROS_DEBUG("invalid parameter in construct func.");
    }
  }
  CaPolicy::~CaPolicy()
  {

  }
  
  geometry_msgs::Point32 Obstacle::getObstacleCenterPos()
  {
    fPoint32 p;
    p.x = (pos_min_.x + pos_max_.x)/2.0;
    p.y = (pos_min_.y + pos_max_.y)/2.0;
    p.z = (pos_min_.z + pos_max_.z)/2.0;
    
    return p;
  }

} //namespace




