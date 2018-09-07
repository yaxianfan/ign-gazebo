/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_SYSTEMS_RENDERING_HH_
#define IGNITION_GAZEBO_SYSTEMS_RENDERING_HH_

#include <memory>
#include <vector>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declarations.
  class RenderingPrivate;

  /// \class Rendering Rendering.hh ignition/gazebo/systems/Rendering.hh
  /// \brief System which handles rendering.
  class IGNITION_GAZEBO_VISIBLE Rendering: public System
  {
    /// \brief Constructor
    public: explicit Rendering();

    /// \brief Destructor
    public: virtual ~Rendering();

    // Documentation inherited
    public: void Init(
                std::vector<EntityQueryCallback> &_cbs) override final;

    /// \brief Private data pointer.
    private: std::unique_ptr<RenderingPrivate> dataPtr;
  };
  }
}
}
}
#endif
