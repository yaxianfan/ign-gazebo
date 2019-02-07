/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef IGNITION_GAZEBO_COMPONENTS_ANGULARVELOCITY_HH_
#define IGNITION_GAZEBO_COMPONENTS_ANGULARVELOCITY_HH_

#include <ignition/math/Vector3.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include "ignition/gazebo/components/SimpleWrapper.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains angular velocity of an entity
  /// represented by ignition::math::Vector3d.
  using AngularVelocity =
    SimpleWrapper<math::Vector3d, class AngularVelocityTag>;

  /// \brief A component type that contains angular velocity of an entity in the
  /// world frame represented by ignition::math::Vector3d.
  using WorldAngularVelocity =
      SimpleWrapper<math::Vector3d, class WorldAngularVelocityTag>;
}
}
}
}
#endif
