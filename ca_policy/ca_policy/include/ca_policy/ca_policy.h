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

#ifndef ICA_CA_POLICY_H
#define ICA_CA_POLICY_H

#include <ros/ros.h>
#include <vector>
#include <math.h>

namespace intelligent_ca
{
/** @brief CA Policy description.
 * This class stores any information about CA Policy.
 * each policy holds a name and a config file.
 */
class CaPolicy
{
public:
  CaPolicy();
  CaPolicy(const std::string& name, const std::string& config);
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
  std::string getConfiguration();

  /** @brief  Set policy name
   *  @param[in] name The policy name to be set.
   *  @return true if successfully set, otherwise false.
   */
  bool setPolicyName(const std::string& name);

  /** @brief Get policy name.
   *  @return String storing policy name.
   */
  std::string getPolicyName();

  virtual void executeReconfig(){};
  virtual void executeLED(std::string type){};

protected:
  // ros::NodeHandle nh_;
  std::string config_file_;  ///@brief configuration file path
  std::string name_;         ///@brief CA Policy name, "safety", "robot", "normal" would be current option.
};

}  // namespace
#endif
