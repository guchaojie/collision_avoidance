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
#include <iostream>
//#include "ca_policy/ca_policy.h"
#include "ca_policy/object_frame.h"
#include <object_bridge_msgs/ObjectMerged.h>
#include <object_bridge_msgs/SocialObject.h>
#include <object_bridge_msgs/SocialObjectsInFrame.h>

namespace intelligent_ca
{
CaObjectFrame::CaObjectFrame() :
    nh_("/intelligent_ca/"), published_(false)
{
  merged_objects_pub_ = nh_.advertise<object_bridge_msgs::ObjectsInFrameMerged>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<object_bridge_msgs::SocialObjectsInFrame>(kTopicSocialObjectInFrame, 1);

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
  social_object_pub_ = nh_.advertise<object_bridge_msgs::SocialObjectsInFrame>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  objects_merged_.clear();
}

CaObjectFrame::CaObjectFrame(ros::Time& stamp, std::string& frame_id, ros::NodeHandle nh) :
    nh_(nh), published_(false)
{
  merged_objects_pub_ = nh_.advertise<object_bridge_msgs::ObjectsInFrameMerged>(kTopicObjectsInFrame, 1);
  social_object_pub_ = nh_.advertise<object_bridge_msgs::SocialObjectsInFrame>(kTopicSocialObjectInFrame, 1);

  initParameter();

  objects_detected_.clear();
  objects_tracked_.clear();
  objects_localized_.clear();

  objects_merged_.clear();
  stamp_ = stamp;
  tf_frame_id_ = frame_id;
}
CaObjectFrame::~CaObjectFrame()
{
}

void CaObjectFrame::initParameter()
{
  nh_.param("social_msg_enabled", social_msg_enabled_, true);
  nh_.param("merged_op_msg_enabled", merged_op_msg_enabled_, true);
  nh_.param("posibility_threshold", posibility_threshold_, 0.1);

  is_merging_ = false;

  /**< @todo TODO: get params for social_filter_ */
  social_filter_.clear();
  social_filter_.push_back("person");
}

void CaObjectFrame::addVector(const DetectionVector& vector)
{
  ROS_INFO("add detection vector ... ");
  objects_detected_ = vector;
  mergeObjects();
  ROS_INFO("size of  objests_detected: %lu", objects_detected_.size());
}

void CaObjectFrame::addVector(const TrackingVector& vector)
{

  objects_tracked_ = vector;
  mergeObjects();

}

void CaObjectFrame::addVector(const LocalizationVector& vector)
{

  objects_localized_ = vector;
  mergeObjects();

}

void CaObjectFrame::mergeObjects()
{
  if (is_merging_)
  {
    ROS_INFO("Objects is already merging...");
    return;
  }

  ROS_INFO("published:%s, data ready:%s", published_ ? "true" : "false", isDataReady() ? "true" : "false");
  if (published_ || !isDataReady())
  {
    ROS_WARN("Already published or data not ready. Do nothing");
    return;
  }

  is_merging_ = true;
  for (DetectionVector::iterator it = objects_detected_.begin(); it != objects_detected_.end(); ++it)
  {
    if(it->object.probability < posibility_threshold_){
      continue;
    }

    ObjectRoi roi = it->roi;
    MergedObject merged_obj;
    TrackingObjectInBox track_obj;
    LocalizationObjectInBox loc_obj;

    bool result = findTrackingObjectByRoi(roi, track_obj);
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
  is_merging_ = false;
}

bool CaObjectFrame::findMergedObjectById(const int id, MergedObject& out)
{
  ObjectMergedVector temp_objects = objects_merged_;
  for (auto t : temp_objects)
  {
    if (t.id == id)
    {
      out = t;
      return true;
    }
  }
  return false;
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
  if (published_) {
    ROS_INFO("Merged objects have been already published, do nothing");
    return false;
  }

  if (merged_op_msg_enabled_) {
    ObjectMergedMsg msg;
    msg.header.frame_id = tf_frame_id_;
    msg.header.stamp = stamp_;
    msg.objects = objects_merged_;
    merged_objects_pub_.publish(msg);
  }

  if (social_msg_enabled_) {

      SoicalObjectVector socials;
      socials.clear();

      if (!objects_merged_.empty()) {
        for (auto ob : objects_merged_)
        {
          if (!isSocialObject(ob))
            break;

          SocialObject so;
          so.id = ob.id;
          so.name = ob.type;
          geometry_msgs::Point32 c = getCentroid(ob);
          so.position.x = c.x;
          so.position.y = c.y;
          so.position.z = c.z;
          so.velocity = ob.velocity;
          so.reliability = ob.probability;
          so.tagnames.clear(); /**< Not used */
          so.tags.clear(); /**< Not used */
          socials.push_back(so);
        } 
      }

      SocialObjectsInFrameMsg msg;
      msg.header.frame_id = tf_frame_id_;
      msg.header.stamp = stamp_;
      msg.objects = socials;
      social_object_pub_.publish(msg);
   }

   setFlagPublished(true);

   return true;
}

bool CaObjectFrame::isSocialObject(MergedObject& ob)
{
  for (auto f : social_filter_)
  {
    if (ob.type.find(f) != std::string::npos)
    {
      return true;
    }
  }

  return false;
}

void CaObjectFrame::setFlagPublished(bool state)
{
  published_ = state;
}
} // namespace
