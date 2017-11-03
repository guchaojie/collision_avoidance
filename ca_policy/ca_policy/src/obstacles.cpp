
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
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
}

Obstacles::Obstacles() : nh_("/intelligent_ca/")
{
  frames_.clear();
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
}

Obstacles::~Obstacles()
{
}

void Obstacles::calcVelocity(std::shared_ptr<CaObjectFrame>& frame)
{
  ObjectMergedVector objects = frame->getMergedObjects();
  std::vector<CaObjectFrame> frames = frames_;

  unsigned int size_frames = frames.size();
  for (auto ob : objects)
  {
    /**< Find the latest objects from frames (in reverse order) */
    for(unsigned int i = size_frames; i>0; --i)
    {
      MergedObject out;
      double duration = frames[i-1].getStamp().toSec() - frame->getStamp().toSec();
      if (duration <= 0.0)
      {
        break;
      }
      if(frames[i-1].findMergedObjectByRoi(ob.roi, out))
      {
        geometry_msgs::Point32 from = CaObjectFrame::getCentroid(ob);
        geometry_msgs::Point32 to = CaObjectFrame::getCentroid(out);
        double distance_x = to.x - from.x;
        double distance_y = to.y - from.y;
        double distance_z = to.z - from.z;

        /**< @todo, double check it is set correctly */
        ob.velocity.x = distance_x / duration;
        ob.velocity.y = distance_y / duration;
        ob.velocity.z = distance_z / duration;
        break;
      }

    }
  }
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
  calcVelocity(frame);
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
