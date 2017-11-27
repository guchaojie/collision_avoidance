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

#ifndef ICA_OBJECT_MERGER_H
#define ICA_OBJECT_MERGER_H

#include <ros/ros.h>
#include <vector>

#include "ca_policy/ca_policy_manager.h"
#include "ca_policy/consts.h"
#include "ca_policy/obstacles.h"

namespace intelligent_ca
{

/** @brief Merging camera related messages into one, with more info(currently velocity) added.
 *         When a given message is received:
 *         1. The class searches the corresponding frame by the same frame_id and stamp, if no, creates a new frame.
 *         2. Add the message to the frame which filtered out in step1.
 *         3. Publish the ready frame(s).
 *         4. Clean the frames which are not in the monitoring window.
 */
class ObjectMerger
{
public:
  ObjectMerger();
  ObjectMerger(ros::NodeHandle& nh);
  //ObjectMerger(const Obstacles* obstacles, const CaPolicyManager* manager);
  virtual ~ObjectMerger();

private:
  void onInit(void);

  void onObjectsReceived(const object_msgs::ObjectsInBoxesConstPtr& detect,
                                      const object_analytics_msgs::TrackedObjectsConstPtr& track,
                                      const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc);
  /** @brief Callback function when receiving messages from AMR_ros_object_msgs package.
   *  @param[in] msg The received message (typed in object_msgs::ObjectsInBoxes)
   */
  void onObjectDetected(const object_msgs::ObjectsInBoxesConstPtr& msg);

  /** @brief Callback function when receiving messages from object_pipeline package.
   *  @param[in] msg The received message (typed in object_analytics_msgs::TrackedObjects)
   */
  void onObjectTracked(const object_analytics_msgs::TrackedObjectsConstPtr& msg);

  /** @brief Callback function when receiving messages from object_pipeline package.
   *  @param[in] msg The received message (typed in object_analytics_msgs::ObjectsInBoxes3D)
   */
  void onObjectLocalized(const object_analytics_msgs::ObjectsInBoxes3DConstPtr& msg);

  //std::shared_ptr<Obstacles> pObstacle_;
  ros::NodeHandle nh_;

  ros::Subscriber detection_sub_; /// the subscriber of detection messages
  ros::Subscriber tracking_sub_; /// the subscriber of tracking messages
  ros::Subscriber localization_sub_; /// the subscriber of localization messages

  std::shared_ptr<Obstacles> frames_; /// the frames storing all obstacles' info

  /**< Object Messages, which are initialized from parameter configurer.*/
  std::string msg_object_detection_;
  std::string msg_object_tracking_;
  std::string msg_object_localization_;
};

} // namespace
#endif
