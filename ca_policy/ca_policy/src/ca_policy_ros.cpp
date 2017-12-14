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

#include "ca_policy/ca_policy_ros.h"
#include "ca_policy/consts.h"
#include <ca_policy_msgs/CaPolicy.h>

namespace intelligent_ca
{

CaPolicyRos::CaPolicyRos(ros::NodeHandle& nh) :
    node_handler_(nh), last_set_time_(0)
{
  ROS_INFO("ENTER CaPolicyRos Constructor...");
  init();
}

CaPolicyRos::~CaPolicyRos()
{
}

void CaPolicyRos::init()
{
  if (!node_handler_.hasParam("CaPolicies"))
  {
    ROS_INFO("No policies are set in parameter file. EXIT!");
    return;
  }
  XmlRpc::XmlRpcValue my_list;
  node_handler_.getParam("CaPolicies", my_list);
  for (int32_t i = 0; i < my_list.size(); ++i)
  {
    std::string pname = static_cast<std::string>(my_list[i]["name"]);
    std::string config = static_cast<std::string>(my_list[i]["config"]);
    ROS_INFO("Using CA Policy \"%s:%s\"", pname.c_str(), config.c_str());
    ROS_INFO("Using CA Policy \"%s:%s\"", pname.c_str(), config.c_str());

    // CaPolicy policy(pname, config);
    std::shared_ptr<CaPolicy> policy = policy_builder_.createInstance(pname);
    policy->setConfiguration(config);
    policy_manager_.addPolicy(pname, policy);
  }
  policy_manager_.setCurrentPolicy("normal");

  vision_obj_sub_ = node_handler_.subscribe(kTopicSocialObjectInFrame, 10, &CaPolicyRos::onObjectReceived, this);
  ca_policy_pub_ = node_handler_.advertise < ca_policy_msgs::CaPolicy > (kTopicCaPolicy, 1);

  node_handler_.param("max_detection_distance", max_detection_distance_, 5.0);
  node_handler_.param("min_interval", min_interval_, 2.0);
}

void CaPolicyRos::onObjectReceived(const object_bridge_msgs::SocialObjectsInFrameConstPtr& msg)
{
  ROS_INFO("RECEIVE Object SocialObjectsInFrameConstPtr topic...");

  ros::Time now = ros::Time::now();
  bool social = false;

  if (!msg->objects.empty()) {
    for (auto obj : msg->objects)
    {
       geometry_msgs::Point p = obj.position;
       if ((sqrt(p.x * p.x + p.y * p.y + p.z * p.z) < max_detection_distance_) &&
         obj.name == "person") {
         social = true;
         break;
       }
    } 
  }

  ca_policy_msgs::CaPolicy pmsg;
  pmsg.header.stamp = now;
  pmsg.header.frame_id = msg->header.frame_id;
  pmsg.robot_id = 0;

  if((social && policy_manager_.getCurrentPolicy() == "social")
      || (!social && policy_manager_.getCurrentPolicy() == "normal"))
  {
    last_set_time_ = now;
  }
  double duration = now.toSec() - last_set_time_.toSec();
  if (social && policy_manager_.getCurrentPolicy() != "social" && duration > min_interval_)
  {
    policy_manager_.setCurrentPolicy("social");
    last_set_time_ = now;

    pmsg.id = ca_policy_msgs::CaPolicy::CAPOLICY_PEOPLE;
    pmsg.name = "social";
    ca_policy_pub_.publish(pmsg);

  }

  if (!social && policy_manager_.getCurrentPolicy() != "normal" && duration > min_interval_)
  {
    policy_manager_.setCurrentPolicy("normal");
    last_set_time_ = now;
    pmsg.id = ca_policy_msgs::CaPolicy::CAPOLICY_NORMAL;
    pmsg.name = "normal";
    ca_policy_pub_.publish(pmsg);
  }
}

} // namespace
