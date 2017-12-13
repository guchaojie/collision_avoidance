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
  ros::NodeHandle nh_;
};

}  // namespace
#endif
