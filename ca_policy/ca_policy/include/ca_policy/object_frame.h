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

#ifndef ICA_CA_OBJECT_FRAME_H
#define ICA_CA_OBJECT_FRAME_H

#include <ros/ros.h>
#include <vector>
#include <dynamic_reconfigure/server.h>
//#include <ca_policy/CaObjectFrameConfig.h>

#include "ca_policy/consts.h"

namespace intelligent_ca
{
/** @brief This class merges topics from object pipeline component, and publish
 * the merged topics.
 * This class stores and manages the objects tracked in one camera frame.
 */
class CaObjectFrame
{
public:
  CaObjectFrame();
  CaObjectFrame(ros::NodeHandle nh);
  CaObjectFrame(ros::Time& stamp, std::string& frame_id, ros::NodeHandle nh=ros::NodeHandle("~"));
  virtual ~CaObjectFrame();

  // init(ObjectVector& vector);

  /** @brief add detection vector (got from object pipeline) to the object frame.
   *  @param[in] vector detection vector storing detection info calculated by object module.
   */
  void addVector(const DetectionVector& vector);

  /** @brief add tracking vector (got from object pipeline) to the object frame.
   *  @param[in] vector Tracking vector storing detection info calculated by object module.
   */
  void addVector(const TrackingVector& vector);

  /** @brief add Localization vector (got from object pipeline) to the object frame.
   *  @param[in] vector Localication vector storing detection info calculated by object module.
   */
  void addVector(const LocalizationVector& vector);

  /** @brief Publish messages with merged object topics.
   *  @return true if correctly published;
   *          false otherwise.
   */
  bool publish();

  /** @brief Check if object information is enough for further calculating.
   *         Object information is the one got from object module, such as Detection Vector, Tracking Vector and
   *         Localization Vector.
   *         The function is normally invoked before generate or publish the merged topics.
   *         @return true if data ready, otherwise false.
   */
  bool isDataReady()
  {
    ROS_ERROR("Data Ready ==detection:%s==tracking:%s==localization:%s",
              objects_detected_.empty()?"false":"true",
                  objects_tracked_.empty()?"false":"true",
                      objects_localized_.empty()?"false":"true"
    );
    return !objects_detected_.empty() && !objects_tracked_.empty() && !objects_localized_.empty();
  };

  /** @brief Get transform frame id, which is used for message generation.
   *         The class stores the frame id getting from object topics.
   *  @return the string with content of transform frame id.
   */
  std::string getTfFrameId()
  {
    return tf_frame_id_;
  };

  /** @brief  Get time stamp, which is used for message generation.
   *          The class stores the timestamp getting from object topics.
   *  @return The ros::Time value of message.
   */
  ros::Time getStamp()
  {
    return stamp_;
  };

  /** @brief Merge separate info of detection, tracking and localization into one topic.
   */
  void mergeObjects();

  /** @brief find the merged object which has the same ROI as the param roi.
   *  @param[in] roi Region of interest within an image, which is used to identify the different objects in an image.
   *  @param[out]track The merged object if found.
   *  @return true if found, otherwise false.
   */
  bool findMergedObjectByRoi(const ObjectRoi& roi, MergedObject& track);
  bool findMergedObjectById(const int id, MergedObject& out);

  ObjectMergedVector& getMergedObjects(){return objects_merged_;};

  /** @brief Get the centroid of an object [Static Function]
   *  @param[in] ob The object to be calculated.
   *  @return The centroid point of the given object.
   */
  static geometry_msgs::Point32 getCentroid(MergedObject& ob)
  {
    geometry_msgs::Point32 out;
    out.x = (ob.min.x + ob.max.x)/2;
    out.y = (ob.min.y + ob.max.y)/2;
    out.z = (ob.min.z + ob.max.z)/2;
    return out;
  }

  /** @brief Check if a merged object is social object.
   *  As current design, social object list:
   *  1. People
   *  2. Robot base
   *  @param[in] ob The merged object (from object pipeline) to be checked.
   *  @return true if the given object is in social type, otherwise false.
   *
   *  @todo, TODO, currently any Merged Object is looked as social object, so simply return true;
   */
  static bool isSocialObject(MergedObject& ob) {return true;};

private:
  // void publishObjectsInGroup();

  /** @brief find the tracking object from the tracking vector which has the same ROI.
   *  @param[in] roi Region of interest within an image, which is used to identify the different objects in an image.
   *  @param[out]track The Tracking object if found.
   *  @return true if found, otherwise false.
   */
  bool findTrackingObjectByRoi(const ObjectRoi& roi, TrackingObjectInBox& track);

  /** @brief find the localization object from the tracking vector which has the same ROI.
   *  @param[in] roi Region of interest within an image, which is used to identify the different objects in an image.
   *  @param[out]track The Tracking object if found.
   *  @return true if found, otherwise false.
   */
  bool findLocalizationObjectByRoi(const ObjectRoi& roi, LocalizationObjectInBox& loc);

  /** @brief Set flag for publish status in case to publish twice.
   *  @param[in] state Publish state to be set in the class.
   */
  void setFlagPublished(bool state);

  void initParameter(void);

  //void configure(ca_policy1::CaObjectFrameConfig &config, uint32_t level);

  ///@brief transform frame id which is archived from object topics and is generated for merged topics.
  std::string tf_frame_id_;

  ///@brief time stamp which is archived from object topics and is generated for merged topics.
  ros::Time stamp_;

  ros::NodeHandle nh_;

  ///@brief vectors storing object info(detection, tracking and localization, got from object topics).
  DetectionVector objects_detected_;
  TrackingVector objects_tracked_;
  LocalizationVector objects_localized_;

  ros::Publisher merged_objects_pub_; ///@brief ros publisher for merged object topic.
  ros::Publisher social_object_pub_; ///@brief ros publisher for social object topic.
  ObjectMergedVector objects_merged_; ///@brief vector storing the merged objects.
  bool published_; ///@brief published status for the merged topic vector. true means published.

  bool social_msg_enabled_;
  bool merged_op_msg_enabled_;
  //dynamic_reconfigure::Server<ca_policy1::CaObjectFrameConfig>* server_;
  //dynamic_reconfigure::Server<ca_policy1::CaObjectFrameConfig>::CallbackType cb_reconfigure_;

  bool is_merging_;
};

}  // namespace
#endif
