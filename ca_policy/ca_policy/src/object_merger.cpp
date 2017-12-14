
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

#include <ros/ros.h>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "ca_policy/object_merger.h"

namespace intelligent_ca
{

ObjectMerger::ObjectMerger() : nh_("~")
{
  ROS_INFO("ENTER default ObjectMerger Constructor...");

  onInit();
}

ObjectMerger::ObjectMerger(ros::NodeHandle& nh) : nh_(nh)
{
  ROS_INFO("ENTER ObjectMerger Constructor...");

  onInit();
}

void ObjectMerger::onInit()
{
  nh_.param("msg_object_detection", msg_object_detection_, kTopicObjectDetection);
  nh_.param("msg_object_tracking", msg_object_tracking_, kTopicObjectTracking);
  nh_.param("msg_object_localization", msg_object_localization_, kTopicObjectLocalization);

  f_detection_sub_ = std::unique_ptr<FilteredDetection>(new FilteredDetection(nh_, msg_object_detection_, 1));
  f_tracking_sub_ = std::unique_ptr<FilteredTracking>(new FilteredTracking(nh_, msg_object_tracking_, 1));
  f_localization_sub_ = std::unique_ptr<FilteredLocalization>(new FilteredLocalization(nh_, msg_object_localization_, 1));
  sync_sub_ = std::unique_ptr<FilteredSync>( new FilteredSync(*f_detection_sub_, *f_tracking_sub_, *f_localization_sub_, 10));
  sync_sub_->registerCallback(boost::bind(&ObjectMerger::onObjectsReceived,this, _1, _2, _3));

  frames_ = std::make_shared<Obstacles>(nh_);
  ROS_INFO("message_detction:%s, tracking:%s, localization:%s", msg_object_detection_.c_str(),
            msg_object_tracking_.c_str(), msg_object_localization_.c_str());
}

ObjectMerger::~ObjectMerger()
{
}


void ObjectMerger::onObjectsReceived(const object_msgs::ObjectsInBoxesConstPtr& detect,
                                    const object_analytics_msgs::TrackedObjectsConstPtr& track,
                                    const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc)
{
  if(loc->header.frame_id != track->header.frame_id || track->header.frame_id != detect->header.frame_id
      || loc->header.frame_id != detect->header.frame_id)
  {
    return;
  }

  frames_->processFrame(detect, track, loc);
}

}  // namespace
