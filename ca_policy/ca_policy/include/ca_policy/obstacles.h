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
#ifndef ICA_CA_OBSTACLES_H
#define ICA_CA_OBSTACLES_H

#include <geometry_msgs/Point32.h>
#include <math.h>
#include <ros/ros.h>
#include <vector>

#include "object_frame.h"
#include "ca_policy/consts.h"

namespace intelligent_ca
{
constexpr int kDefaultMaxFrames=20;

class Obstacles
{
public:
  Obstacles();
  Obstacles(ros::NodeHandle nh);
  virtual ~Obstacles();

  void processFrame(const object_msgs::ObjectsInBoxesConstPtr& detect,
                    const object_analytics_msgs::TrackedObjectsConstPtr& track,
                    const object_analytics_msgs::ObjectsInBoxes3DConstPtr& loc);

  void calcVelocity(CaObjectFrame& frame);

  /** @brief Search and return the instance of a frame by the given frame_id and time stamp.
   *         If no cached instance, create a new one and return it.
   *  @param[in] stamp    The time stamp when the frame is taken, which is used to search the frame.
   *  @param[in] frame_id The id the frame, which is used to search the frame.
   *  @return the pointer to the frame instance.
   */
  std::shared_ptr<CaObjectFrame> getInstance(ros::Time stamp, std::string frame_id);

  /** @brief Add Object Frame into obstacles list.
   *  @param[in] frame The object frame to be added.
   *  @return    true  If succeeded in adding,
   *             false otherwise.
   */
  bool addObjectFrame(CaObjectFrame*& frame);

  /** @brief Find and return the object frame by the given time stamp and frame id.
   *  @param[in]  stamp     The time stamp when the frame is taken, which is used to search the frame.
   *  @param[in]  frame_id  The id the frame, which is used to search the frame.
   *  @param[out] frame_out The pointer to the frame if found
   *  @return     true      if succeeded in adding,
   *              false     otherwise.
   */
  bool findObjectFrame(ros::Time stamp, std::string frame_id, std::shared_ptr<CaObjectFrame>& frame_out);


  /** @brief Clean the cached frames and remove the old ones if the size of frames is over the threshold. */
  void clearOldFrames();

private:
  std::vector<CaObjectFrame> frames_;
  int max_frames_;  /**< The number of frames to be archived in memory. */
  bool velocity_enabled;
  ros::NodeHandle nh_;
};

}  // namespace
#endif
