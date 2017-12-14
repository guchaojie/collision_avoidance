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

#ifndef ICA_CA_BRIDGE_H
#define ICA_CA_BRIDGE_H

#include <ros/ros.h>
#include "ca_policy_manager.h"
#include "obstacles.h"
#include "ca_policy/obstacle.h"

namespace intelligent_ca
{
class CaBridge:
{
public:
  CaBridge();
  CaBridge(const Obstacles* obstacles, const CaPolicyManager* manager);
  virtual ~ObstacleFromCamera();

  bool calculateCaPolicy();
  bool publishCaPolicy();

private:
  std::shared_ptr<Obstacles> pObstacle_;
  std::shared_ptr<CaPolicyManager> pPolicyManager_;
};

}  // namespace
#endif
