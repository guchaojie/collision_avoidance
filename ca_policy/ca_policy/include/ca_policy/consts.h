/******************************************************************************
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

#ifndef ICA_CONSTS_H
#define ICA_CONSTS_H

#include <ros/ros.h>
#include <vector>
//#include <sensor_msgs/people.h>
#include <object_analytics_msgs/ObjectInBox3D.h>
#include <object_analytics_msgs/ObjectsInBoxes3D.h>
#include <object_analytics_msgs/TrackedObject.h>
#include <object_analytics_msgs/TrackedObjects.h>
#include <object_msgs/Object.h>
#include <object_msgs/ObjectInBox.h>
#include <object_msgs/ObjectsInBoxes.h>

#include <object_bridge_msgs/ObjectMerged.h>
#include <object_bridge_msgs/ObjectsInFrameMerged.h>
#include <object_bridge_msgs/SocialObject.h>
#include <object_bridge_msgs/SocialObjectsInFrame.h>

#include <ca_policy_msgs/CaPolicy.h>

namespace intelligent_ca
{
using DetectionObject = object_msgs::Object;
using DetectionObjectInBox = object_msgs::ObjectInBox;
using TrackingObjectInBox = object_analytics_msgs::TrackedObject;
using LocalizationObjectInBox = object_analytics_msgs::ObjectInBox3D;
using MergedObject = object_bridge_msgs::ObjectMerged;
using SocialObject = object_bridge_msgs::SocialObject;

using DetectionMsg = object_msgs::ObjectsInBoxes;
using TrackingMsg = object_analytics_msgs::TrackedObjects;
using LocalizationMsg = object_analytics_msgs::ObjectsInBoxes3D;
using ObjectMergedMsg = object_bridge_msgs::ObjectsInFrameMerged;
using SocialObjectsInFrameMsg = object_bridge_msgs::SocialObjectsInFrame;

using DetectionVector = std::vector<DetectionObjectInBox>;
using TrackingVector = std::vector<TrackingObjectInBox>;
using LocalizationVector = std::vector<LocalizationObjectInBox>;
using ObjectMergedVector = std::vector<MergedObject>;
using SoicalObjectVector = std::vector<SocialObject>;

using ObjectRoi = sensor_msgs::RegionOfInterest;

/*
constexpr char* kTopicObjectDetection = "/todo"; // TODO
constexpr char* kTopicObjectLocalization = "/result_3d"; // TODO
constexpr char* kTopicCaPolicy = "/ca_policy";
constexpr char* kTopicObjectsInFrame = "object_merged";
constexpr char* kTopicSocialObjectInFrame = "";
*/
const std::string kTopicObjectDetection = "/object_pipeline/detection";
const std::string kTopicObjectTracking = "/object_pipeline/tracking";
const std::string kTopicObjectLocalization = "/object_pipeline/localization";

const std::string kTopicCaPolicy = "ca_policy";
const std::string kTopicObjectsInFrame = "object_merged";
const std::string kTopicSocialObjectInFrame = "/vision_object/social_object";

//bool ica_debug_ = true;
/**
bool isEqual(const ObjectRoi& left, const ObjectRoi& right)
{
  if (left.x_offset == right.x_offset && left.y_offset == right.y_offset && left.width == right.width
      && left.height == right.height)
  {
    return true;
  }
  return false;
}*/

} // namespace
#endif
