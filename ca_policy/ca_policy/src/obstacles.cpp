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

#include <vector>

#include <math.h>
#include <ros/ros.h>
#include "ca_policy/obstacles.h"

namespace intelligent_ca
{
Obstacles::Obstacles(ros::NodeHandle nh) :
    nh_(nh)
{
  frames_.clear();
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
}

Obstacles::Obstacles() :
    nh_("/intelligent_ca/")
{
  frames_.clear();
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
}

Obstacles::~Obstacles()
{
}

void Obstacles::calcVelocity(CaObjectFrame& frame)
{
  std::vector<CaObjectFrame> frames = frames_;
  unsigned int size_frames = frames.size();
  ROS_INFO("Caculating VELOCITY (total %d frames...", size_frames);
  for (ObjectMergedVector::iterator ob = frame.getMergedObjects().begin(); ob != frame.getMergedObjects().end(); ++ob)
  {
    ROS_INFO("....process Object.");
    geometry_msgs::Point sum_vel;
    int sum_count = 0;
    sum_vel.x = sum_vel.y = sum_vel.z = 0.0;

    /**< Find the latest objects from frames (in reverse order) */
    for (unsigned int i = size_frames; i > 0; --i)
    {

      double duration = frames[i - 1].getStamp().toSec() - frame.getStamp().toSec();
      if (duration == 0.0)
      {
        continue;
      }
      ROS_INFO("......compare with other frames...");
      MergedObject out;
      if (frames[i - 1].findMergedObjectById(ob->id, out))
      {
        ROS_INFO("Finding the VEL Candidate");
        geometry_msgs::Point32 from = CaObjectFrame::getCentroid(*ob);
        geometry_msgs::Point32 to = CaObjectFrame::getCentroid(out);
        double distance_x = to.x - from.x;
        double distance_y = to.y - from.y;
        double distance_z = to.z - from.z;

        /**< @todo, double check it is set correctly */
        sum_vel.x += distance_x / duration;
        sum_vel.y += distance_y / duration;
        sum_vel.z += distance_z / duration;
        sum_count++;
        break;
      }
    }
    /**< @todo, double check it is set correctly */
    if (sum_count > 0)
    {
      ROS_INFO("Setting velocity...");
      ob->velocity.x = sum_vel.x / sum_count;
      ob->velocity.y = sum_vel.y / sum_count;
      ob->velocity.z = sum_vel.z / sum_count;
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

void Obstacles::processFrame(const object_msgs::ObjectsInBoxesConstPtr& detect,
                          const object_analytics_msgs::TrackedObjectsConstPtr& track,
                          const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc)
{
  /**< make sure old frames are already cleared first. */
  clearOldFrames();

  ros::Time stamp = detect->header.stamp;
  std::string frame_id = detect->header.frame_id;

  CaObjectFrame new_frame(stamp, frame_id, nh_);
  new_frame.addVector(detect->objects_vector);
  new_frame.addVector(track->tracked_objects);
  new_frame.addVector(loc->objects_in_boxes);
  bool velocity_enabled;
  nh_.param("velocity_enabled", velocity_enabled, true);
  if(velocity_enabled)
  {
    calcVelocity(new_frame);
  }
  new_frame.publish();
  frames_.push_back(new_frame);
}

std::shared_ptr<CaObjectFrame> Obstacles::getInstance(ros::Time stamp, std::string frame_id)
{

  ROS_INFO("Finding frame: FrameID=%s, Stamp=%10.8f", frame_id.c_str(), stamp.toSec());
  try
  {
    int size = frames_.size();
    for (int i = 0; i < size; ++i)
    {
      if (frames_[i].getTfFrameId() == frame_id && frames_[i].getStamp() == stamp)
      {
        ROS_INFO("Founded!!");
        return std::shared_ptr<CaObjectFrame>(&frames_[i]);
      }
    }

    ROS_INFO("Not found! Then Created new");
    CaObjectFrame new_frame(stamp, frame_id, nh_);
    //std::shared_ptr<CaObjectFrame> frame(new CaObjectFrame(stamp, frame_id, nh_));
    frames_.push_back(new_frame);
    ROS_INFO("Created!");
    return std::shared_ptr<CaObjectFrame>(&frames_.back());

  }
  catch (...)
  {
    ROS_INFO("Failed when getInstance from Obstacles");
    CaObjectFrame new_frame(stamp, frame_id, nh_);
    frames_.push_back(std::move(new_frame));
    return std::shared_ptr<CaObjectFrame>(std::move(&new_frame));
  }

}

bool Obstacles::findObjectFrame(ros::Time stamp, std::string frame_id, std::shared_ptr<CaObjectFrame>& frame_out)
{
  bool ret = false;

  int size = frames_.size();

  for (int i = 0; i < size; ++i)
  {
    if (frames_[i].getTfFrameId() == frame_id && frames_[i].getStamp() == stamp)
    {
      frame_out = std::shared_ptr<CaObjectFrame>(&frames_[i]);
      ret = true;
      break;
    }
  }

  return ret;
}

bool Obstacles::addObjectFrame(CaObjectFrame*& frame)
{
  bool ret = false;

  if (frame != nullptr)
  {
    frames_.push_back(*frame);
    ret = true;
  }

  return ret;
}
} // namespace
