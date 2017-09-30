/******************************************************************************
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
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>

#include "object_frame.h"

#ifndef ICA_CA_OBSTACLES_H
#define ICA_CA_OBSTACLES_H

namespace intelligent_ca {


class Obstacles
{
public:
  Obstacles();
  Obstacles(ros::NodeHandle nh);
  virtual ~Obstacles();
  std::shared_ptr<CaObjectFrame> getInstance(ros::Time stamp, std::string frame_id);
  bool addObjectFrame(const CaObjectFrame*& frame);
  bool findObjectFrame(ros::Time stamp, std::string frame_id, std::shared_ptr<CaObjectFrame> 
frame_out);
  //std::vector<CaObjectFrame> getAllFrames();
  //virtual geometry_msgs::Point32 getObstacleCenterPos();
  bool calcVelocity(void);
  void publish(std::shared_ptr<CaObjectFrame>& frame);
  void clearOldFrames();
  
private:
  std::vector<CaObjectFrame> frames_;
  int max_frames_; ///@brief The number of frames to be archived in memory.
  ros::NodeHandle nh_;
};

} //namespace
#endif
