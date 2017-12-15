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

#pragma once
#ifndef ICA_CONSTS_H
#define ICA_CONSTS_H

#include <ros/ros.h>
#include <vector>
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

const std::string kTopicObjectDetection = "/object_pipeline/detection";
const std::string kTopicObjectTracking = "/object_pipeline/tracking";
const std::string kTopicObjectLocalization = "/object_pipeline/localization";

const std::string kTopicCaPolicy = "ca_policy";
const std::string kTopicObjectsInFrame = "object_merged";
const std::string kTopicSocialObjectInFrame = "/vision_object/social_object";

} // namespace
#endif
