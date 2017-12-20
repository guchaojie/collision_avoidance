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
#ifndef ICA_CA_POLICY_H
#define ICA_CA_POLICY_H

#include <ros/ros.h>
#include <vector>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>

namespace intelligent_ca
{
/**< LED States for Water Robot Base. */
enum LEDState
{
  CA_LED_OFF = 0,
  CA_LED_ON,
  CA_LED_BLINK,
  CA_LED_BREATHE
};

/** @brief CA Policy description.
 * This class stores any information about CA Policy.
 * each policy holds a name and a config file.
 */
class CaPolicy
{
public:
  CaPolicy(const ros::NodeHandle& ros_node);
  // CaPolicy(ros::NodeHandle* ros_node, const std::string& name, const std::string& config);
  virtual ~CaPolicy();

  /** @brief execute the policy*/
  virtual void execute(){};

  /** @brief set policy's configuration file.
   *  @param[in] config the config file path to be set to the Ca Policy.
   *  @return    true if successfully set, otherwise false.
   */
  bool setConfiguration(const std::string& config);

  /** @brief  Get the config file path for the ca policy.
   *  @return The string storing the policy's config file path.
   */
  std::string getConfiguration()
  {
    return config_file_;
  }

  /** @brief  Set policy name
   *  @param[in] name The policy name to be set.
   *  @return true if successfully set, otherwise false.
   */
  bool setPolicyName(const std::string& name);

  /** @brief Get policy name.
   *  @return String storing policy name.
   */
  std::string getPolicyName()
  {
    return name_;
  }

  /** @brief Reconfigure Navigation's paramters */
  virtual void executeReconfig(){};

  /** @brief Set LED's status.
   *  @param[in] state LED status to be set. see enum LEDState..
   */
  void executeLED(LEDState state);

protected:
  ros::NodeHandle nh_;
  std::string config_file_;  ///@brief configuration file path
  std::string name_;         ///@brief CA Policy name, "safety", "robot", "normal" would be current option.

  int luminance_;
  /** Format of led_array_.data:
   *  data[0]: LED state, see enum LEDState.
   *  data[1]: 0, not used.
   *  data[2]: 0, not used.
   *  data[3]: luminance [0, 255]
   */
  std_msgs::UInt8MultiArray led_array_;
  ros::Publisher ca_led_pub_;
};

}  // namespace
#endif
