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

#ifndef ICA_CA_POLICY_ROS_H
#define ICA_CA_POLICY_ROS_H

#include <ros/ros.h>
#include <vector>

#include "ca_policy/ca_policy.h"
#include "ca_policy/ca_policy_manager.h"
#include "ca_policy/ca_policy_builder.h"
#include <ca_policy_msgs/CaPolicy.h>
#include <object_bridge_msgs/SocialObjectsInFrame.h>

namespace intelligent_ca
{


class CaPolicyRos
{
public:
  // CaPolicyRos();
  CaPolicyRos(ros::NodeHandle& nh);
  virtual ~CaPolicyRos();

private:
  /** @brief Callback function when receiving messages from AMR_ros_object_msgs package.
   *  @param[in] msg The received message (typed in object_msgs::ObjectsInBoxes)
   */
  void onObjectReceived(const object_bridge_msgs::SocialObjectsInFrameConstPtr& msg);

  void init();

  ros::NodeHandle node_handler_;

  ros::Subscriber vision_obj_sub_; /// the subscriber of detection messages
  ros::Publisher  ca_policy_pub_;

  CaPolicyManager policy_manager_;
  CaPolicyBuilder policy_builder_;

  double max_detection_distance_;
  double min_interval_;

  ros::Time last_set_time_;
};

} // namespace
#endif
