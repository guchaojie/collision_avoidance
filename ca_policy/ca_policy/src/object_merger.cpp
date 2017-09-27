
/******************************************************************************  *
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
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <object_bridge_msgs/ObjectMerged.h> 
#include <ca_policy/ca_policy.h>
#include <ca_policy/object_merger.h>

namespace intelligent_ca {


  ObjectMerger::ObjectMerger()
  : nh_("~")
  {
    detection_sub_ = nh_.subscribe(kTopicObjectDetection, 10, &ObjectMerger::onObjectDetected, 
this);
    tracking_sub_ = nh_.subscribe(kTopicObjectDetection, 10, &ObjectMerger::onObjectTracked, this);
    localization_sub_ = nh_.subscribe(kTopicObjectLocalization, 10, 
&ObjectMerger::onObjectLocalized, this);
    
    ///We assume the maximum object detected in one camera frame is less than 10
    ///We just store at most 2 frames of topics
    //objects_detected_.reserve(10*2);
    //objects_tracked_.reserve(10*2);
    //objects_localized_.reserve(10*2);

    //nh_.param<std::string>("base_frame", base_frame_, "/base_link");
    
    //ros::ServiceServer service = nh_.advertiseService("obj_merger",&CaPolicy::mergeObjects);
    
    frames_ = std::make_shared<Obstacles>(nh_);
    
  }
  
  ObjectMerger::~ObjectMerger()
  {

  }
  
  void ObjectMerger::onObjectDetected(const DetectionMsg msg)
  {
    std::shared_ptr<CaObjectFrame> frame = frames_->getInstance(msg.header.stamp, 
msg.header.frame_id);
    
    frame->addVector(msg.objects_vector);
    
    publish(frames_, frame);
    frames_.clearOldFrames();
   
  }
  
  void ObjectMerger::onObjectTracked(const TrackingMsg msg)
  {
    std::shared_ptr<CaObjectFrame> frame = frames_->getInstance(msg.header.stamp, 
msg.header.frame_id);
    
    frame->addVector(msg.tracked_objects);
    frames_.publish(frame);
    frames_.clearOldFrames();
    
  }
  
  void ObjectMerger::onObjectLocalized(const LocalizationMsg msg)
  {
    std::shared_ptr<CaObjectFrame> frame = frames_->getInstance(msg.header.stamp, 
msg.header.frame_id);
    
    frame->addVector(msg.objects_in_boxes);
    frames_.publish(frame);
    frames_.clearOldFrames();
  }
  
 

} //namespace


