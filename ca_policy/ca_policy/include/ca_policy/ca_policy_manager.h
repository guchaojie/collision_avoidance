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

#ifndef ICA_CA_POLICY_MANAGER_H
#define ICA_CA_POLICY_MANAGER_H

#include <ros/ros.h>
#include <vector>

#include "ca_policy.h"
#include <ca_policy/ca_policy.h>

namespace intelligent_ca
{
using CaPolicyPair = std::pair<std::string, CaPolicy>;
using CaPolicyVector = std::vector<CaPolicyPair>;

/** @brief This class manages the CA policies in use.
 * add, delete, set policy name, set current policy.
 */
class CaPolicyManager
{
public:
  CaPolicyManager();
  virtual ~CaPolicyManager();

  /** @brief add CA policy by the given name and policy instance.
   *  @param[in] name   The name of the policy to be added.
   *  @param[in] policy Policy to be added.
   *  @return true if successfully added, otherwise false.
   */
  bool addPolicy(const std::string name, const CaPolicy& policy);

  /** @brief delete CA policy by the give policy name.
   *  @param[in] name The name of the policy to be deleted.
   *  @return true if successfully deleted, otherwise false.
   */
  bool deletePolicy(const std::string name);

  // CaPolicy getPolicyByName(const std::string name);

  bool setCurrentPolicy(const std::string name);

private:

  /** @brief Search and return the CA policy by the given name.
   *  @param[in] name The name of the policy to be searched.
   *  @param[out] pair The iterator of the policy found.
   *  @return true if found, otherwise false.
   */
  bool findPolicy(const std::string name, CaPolicyVector::iterator& pair);

  CaPolicyVector policies_; ///@brief CA policy list
  CaPolicyPair current_policy_; ///@brief The working CA policy, which is used to configure navigation.
};

}  // namespace
#endif
