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
//#include <sensor_msgs/people.h>
#include <tf/transform_listener.h>
#include <ros_yolo_msgs/Object.h>
#include <ros_yolo_msgs/ObjectInBox.h>
#include <ros_yolo_msgs/ObjectsInBoxes.h>
#include <object_pipeline_msgs/TrackedObject.h>
#include <object_pipeline_msgs/ObjectInBox3D.h>
#include <object_pipeline_msgs/TrackedObjects.h>
#include <object_pipeline_msgs/ObjectsInBoxes3D.h>

#include <object_bridge_msgs/ObjectMerged.h>
#include <object_bridge_msgs/ObjectsInFrameMerged.h>

#ifndef ICA_CONSTS_H_
#define ICA_CONSTS_H_

namespace intelligent_ca {
  using DetectionObject = ros_yolo_msgs::Object;
  using DetectionObjectInBox = ros_yolo_msgs::ObjectInBox;
  using TrackingObjectInBox = object_pipeline_msgs::TrackedObject;
  using LocalizationObjectInBox = object_pipeline_msgs::ObjectInBox3D;
  
  using DetectionMsg = ros_yolo_msgs::ObjectsInBoxes;
  using TrackingMsg = object_pipeline_msgs::TrackedObjects;
  using LocalizationMsg = object_pipeline_msgs::ObjectsInBoxes3D;
  
  using DetectionVector = std::vector<DetectionObjectInBox>;
  using TrackingVector = std::vector<TrackingObjectInBox>; 
  using LocalizationVector = std::vector<LocalizationObjectInBox>;
  
  using MergedObject = object_bridge_msgs::ObjectMerged;
  using ObjectMergedVector = std::vector<MergedObject>;
  using ObjectMergedMsg = object_bridge_msgs::ObjectsInFrameMerged;
  
  const std::string kTopicObjectDetection = "/todo"; //TODO
  const std::string kTopicObjectTracking = "/todo"; //TODO
  const std::string kTopicObjectLocalization = "/result_3d"; //TODO

  const std::string kTopicCaPolicy = "/ca_policy";
  const std::string kTopicObjectsInFrame = "/object_merged";


} //namespace
#endif
