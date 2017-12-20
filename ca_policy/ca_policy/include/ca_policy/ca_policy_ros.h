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
  CaPolicyRos(ros::NodeHandle& nh);
  virtual ~CaPolicyRos();

private:
  /** @brief Callback function when receiving messages from AMR_ros_object_msgs package.
   *  @param[in] msg The received message (typed in object_msgs::ObjectsInBoxes)
   */
  void onObjectReceived(const object_bridge_msgs::SocialObjectsInFrameConstPtr& msg);

  /** @brief Initialize the parameters and variables. */
  void init();

  ros::NodeHandle node_handler_;

  ros::Subscriber vision_obj_sub_; /**< the subscriber of detection messages. */
  ros::Publisher ca_policy_pub_;   /**< The publisher of ca_policy status. */

  CaPolicyManager policy_manager_;
  CaPolicyBuilder policy_builder_;

  double max_detection_distance_;
  double min_interval_;

  ros::Time last_set_time_;
};

}  // namespace
#endif
