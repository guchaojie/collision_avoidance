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
#ifndef ICA_NORMAL_CA_POLICY_H
#define ICA_NORMAL_CA_POLICY_H

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include "ca_policy/ca_policy.h"

namespace intelligent_ca
{
/** @brief CA Policy description.
 * This class stores any information about CA Policy.
 * each policy holds a name and a config file.
 */
class NormalCaPolicy:public CaPolicy
{
public:
  NormalCaPolicy();
  NormalCaPolicy(const std::string& name, const std::string& config);
  virtual ~NormalCaPolicy();

  /** @brief execute the policy*/
  virtual void execute();

};

}  // namespace
#endif
