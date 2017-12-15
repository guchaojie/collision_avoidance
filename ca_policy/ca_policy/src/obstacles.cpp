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
  nh_.param("velocity_enabled", velocity_enabled, true);

}

Obstacles::Obstacles() :
    nh_("/intelligent_ca/")
{
  frames_.clear();
  nh_.param("max_frames", max_frames_, kDefaultMaxFrames);
  nh_.param("velocity_enabled", velocity_enabled, true);
}

Obstacles::~Obstacles()
{
}

void Obstacles::calcVelocity(CaObjectFrame& frame)
{
  if (!velocity_enabled)
    return;

  std::vector<CaObjectFrame> frames = frames_;
  unsigned int size_frames = frames.size();
  ROS_INFO("Caculating VELOCITY (total %d frames...", size_frames);
  for (ObjectMergedVector::iterator ob = frame.getMergedObjects().begin(); ob != frame.getMergedObjects().end(); ++ob)
  {
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

      MergedObject out;
      if (frames[i - 1].findMergedObjectById(ob->id, out))
      {
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

  if (loc->objects_in_boxes.size() != 0 &&
      track->tracked_objects.size() != 0 &&
      detect->objects_vector.size() != 0) {
    new_frame.addVector(detect->objects_vector);
    new_frame.addVector(track->tracked_objects);
    new_frame.addVector(loc->objects_in_boxes);
    calcVelocity(new_frame);

    frames_.push_back(new_frame);
  }
  
  new_frame.publish();
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
        return std::shared_ptr<CaObjectFrame>(&frames_[i]);
      }
    }

    CaObjectFrame new_frame(stamp, frame_id, nh_);
    //std::shared_ptr<CaObjectFrame> frame(new CaObjectFrame(stamp, frame_id, nh_));
    frames_.push_back(new_frame);
    return std::shared_ptr<CaObjectFrame>(&frames_.back());

  }
  catch (...)
  {
    ROS_WARN("Failed when getInstance from Obstacles");
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
