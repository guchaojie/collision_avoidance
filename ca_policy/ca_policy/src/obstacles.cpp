
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

#include <math.h>
#include <ros/ros.h>
//#include <ca_policy_msgs/People.h>
#include "ca_policy/obstacles.h"
#include "object_bridge_msgs/ObjectMerged.h"

namespace intelligent_ca
{
Obstacles::Obstacles(ros::NodeHandle nh) : nh_(nh)
{
  frames_.clear();
  max_frames_ = 10;  ///@todo, Set the max number of frames in memory with parameter system.
}

Obstacles::Obstacles() : nh_("~")
{
  frames_.clear();
  max_frames_ = 10;  ///@todo, Set the max number of frames in memory with parameter system.
}

Obstacles::~Obstacles()
{
}

bool Obstacles::calcVelocity(void)
{
  /// TODO:

  return true;
}

void Obstacles::clearOldFrames()
{
  int olds = frames_.size() - max_frames_;

  if (olds > 0)
  {
    frames_.erase(frames_.begin(), frames_.begin() + olds);
  }
}

void Obstacles::publish(std::shared_ptr<CaObjectFrame>& frame)
{
  calcVelocity();
  frame->publish();
}

std::shared_ptr<CaObjectFrame> Obstacles::getInstance(ros::Time stamp, std::string frame_id)
{
  std::shared_ptr<CaObjectFrame> frame_out = nullptr;
  try
  {
    bool result = findObjectFrame(stamp, frame_id, frame_out);
    if (!result)
    {
      frame_out = std::make_shared<CaObjectFrame>(nh_);
      frames_.push_back(*frame_out);
    }
  }
  catch (...)
  {
    ROS_ERROR("Failed when getInstance from Obstacles");
    return nullptr;
  }

  return frame_out;
}

bool Obstacles::findObjectFrame(ros::Time stamp, std::string frame_id, std::shared_ptr<CaObjectFrame> frame_out)
{
  bool ret = false;

  for (std::vector<CaObjectFrame>::iterator it = frames_.begin(); it != frames_.end(); ++it)
  {
    if (it->getTfFrameId() == frame_id && it->getStamp() == stamp)
    {
      frame_out = std::shared_ptr<CaObjectFrame>(&(*it));
      ret = true;
      break;
    }
  }

  return ret;
}

bool Obstacles::addObjectFrame(const CaObjectFrame*& frame)
{
  bool ret = false;

  if (frame != nullptr)
  {
    frames_.push_back(*frame);
    ret = true;
  }

  return ret;
}
}  // namespace
