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

//#include <boost/thread/thread.hpp>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>

#include "ca_policy/ca_policy.h"
#include "ca_policy/object_frame.h"
#include <object_bridge_msgs/ObjectMerged.h>
#include <object_bridge_msgs/SocialObject.h>


namespace intelligent_ca
{
CaObjectFrame::CaObjectFrame() :
    nh_("/intelligent_ca/"), published_(false)
{
  merged_objects_pub_ = nh_.advertise<object_bridge_msgs::ObjectsInFrameMerged>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<object_bridge_msgs::SocialObject>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  objects_merged_.clear();
}

CaObjectFrame::CaObjectFrame(ros::NodeHandle nh) :
    nh_(nh), published_(false)
{
  merged_objects_pub_ = nh_.advertise<object_bridge_msgs::ObjectsInFrameMerged>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<object_bridge_msgs::SocialObject>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  objects_merged_.clear();
}

CaObjectFrame::~CaObjectFrame()
{
}

void CaObjectFrame::initParameter()
{
  nh_.param("social_msg_enabled", social_msg_enabled_, true);
  nh_.param("merged_op_msg_enabled", merged_op_msg_enabled_, true);

  /*server_ = new dynamic_reconfigure::Server<ca_policy1::CaObjectFrameConfig>(nh_);
  cb_reconfigure_ = boost::bind(&CaObjectFrame::configure, this, _1, _2);
  server_->setCallback(cb_reconfigure_);*/
}
/*
void CaObjectFrame::configure(ca_policy1::CaObjectFrameConfig &config, uint32_t level)
{
  social_msg_enabled_ = config.social_msg_enabled;
  merged_op_msg_enabled_ = config.merged_op_msg_enabled;
}*/

void CaObjectFrame::addVector(const DetectionVector& vector)
{
  for (auto obj : vector)
  {
    objects_detected_.push_back(obj);
  }
}

void CaObjectFrame::addVector(const TrackingVector& vector)
{
  for (auto obj : vector)
  {
    objects_tracked_.push_back(obj);
  }
}

void CaObjectFrame::addVector(const LocalizationVector& vector)
{
  for (auto obj : vector)
  {
    objects_localized_.push_back(obj);
  }
}

void CaObjectFrame::mergeObjects()
{
  if (published_ || isDataReady())
  {
    ROS_WARN("Already published or data not ready. Do nothing");
    return;
  }

  for (DetectionVector::iterator it = objects_detected_.begin(); it != objects_detected_.end(); ++it)
  {
    ObjectRoi roi = it->roi;
    bool result = false;
    MergedObject merged_obj;
    TrackingObjectInBox track_obj;
    LocalizationObjectInBox loc_obj;

    result = findTrackingObjectByRoi(roi, track_obj);
    if (result)
    {
      result = findLocalizationObjectByRoi(roi, loc_obj);
      if (result)
      {
        merged_obj.min = loc_obj.min;
        merged_obj.max = loc_obj.max;
        merged_obj.id = track_obj.id;
        merged_obj.type = it->object.object_name;
        merged_obj.probability = it->object.probability;
        merged_obj.roi = it->roi;
        merged_obj.velocity.x = merged_obj.velocity.y = merged_obj.velocity.z = -1.0;

        objects_merged_.push_back(merged_obj);
      }
    }

  } // end of for(...)
}

bool CaObjectFrame::findMergedObjectByRoi(const ObjectRoi& roi, MergedObject& out)
{
  ObjectMergedVector temp_objects = objects_merged_;
  for (auto t : temp_objects)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset && roi.width == t.roi.width
        && roi.height == t.roi.height)
    {
      out = t;
      return true;
    }
  }

  return false;
}

bool CaObjectFrame::findTrackingObjectByRoi(const ObjectRoi& roi, TrackingObjectInBox& track)
{
  for (auto t : objects_tracked_)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset && roi.width == t.roi.width
        && roi.height == t.roi.height)
    {
      track = t;
      return true;
    }
  }

  return false;
}

bool CaObjectFrame::findLocalizationObjectByRoi(const ObjectRoi& roi, LocalizationObjectInBox& loc)
{
  for (auto t : objects_localized_)
  {
    if (roi.x_offset == t.roi.x_offset && roi.y_offset == t.roi.y_offset && roi.width == t.roi.width
        && roi.height == t.roi.height)
    {
      loc = t;
      return true;
    }
  }

  return false;
}

bool CaObjectFrame::publish()
{
  if (published_)
  {
    ROS_ERROR("Merged objects have been already published, do nothing");
    return false;
  }

  if (!objects_merged_.empty())
  {
    if (merged_op_msg_enabled_)
    {
      ObjectMergedMsg msg;
      msg.header.frame_id = tf_frame_id_;
      msg.header.stamp = stamp_;
      msg.objects = objects_merged_;
      merged_objects_pub_.publish(msg);
    }

    if(social_msg_enabled_)
    {
      SocialObjectMsg msg;
      msg.header.frame_id = tf_frame_id_;
      msg.header.stamp = stamp_;
      for (auto ob: objects_merged_)
      {
        if(isSocialObject(ob))
        {
          msg.name = ob.type;
          geometry_msgs::Point32 c = getCentroid(ob);
          msg.position.x = c.x;
          msg.position.y = c.y;
          msg.position.z = c.z;
          msg.velocity = ob.velocity;
          msg.reliability = ob.probability;
          msg.tagnames.clear(); /**< Not used */
          msg.tags.clear(); /**< Not used */
          social_object_pub_.publish(msg);
        }
      }

    }
    setFlagPublished(true);

    return true;
  }

  return false;
}

void CaObjectFrame::setFlagPublished(bool state)
{
  published_ = state;
}
} // namespace
