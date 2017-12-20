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

#ifndef ICA_CA_POLICY_BUILDER_H
#define ICA_CA_POLICY_BUILDER_H

#include <ros/ros.h>
#include <vector>

#include "ca_policy/ca_policy.h"
#include "ca_policy/ca_policy_social.h"
#include "ca_policy/ca_policy_normal.h"

namespace intelligent_ca
{
/** @brief This class creates CA Policy instance by the given policy name.
 */
class CaPolicyBuilder
{
public:
  CaPolicyBuilder(){};
  virtual ~CaPolicyBuilder(){};

  /** @brief Create CA policy instance by the given name.
   *  @param[in] name the name of the CA policy to be created.
   *  @return    the shared pointer of the created CA Policy instance.
   */
  std::shared_ptr<CaPolicy> createInstance(const std::string name)
  {
    if (name == "social")
      return std::make_shared<SocialCaPolicy>();

    return std::make_shared<NormalCaPolicy>();
  }
};

}  // namespace
#endif
