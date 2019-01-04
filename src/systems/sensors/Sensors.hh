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
#ifndef IGNITION_GAZEBO_SYSTEMS_SENSORS_HH_
#define IGNITION_GAZEBO_SYSTEMS_SENSORS_HH_

#include <memory>
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
  class SensorsPrivate;

  /// \class Sensors Sensors.hh ignition/gazebo/systems/Sensors.hh
  /// \brief TODO(louise) Have one system for all sensors, or one per
  /// sensor / sensor type?
  class IGNITION_GAZEBO_VISIBLE Sensors:
    public System,
    public ISystemConfigure,
    public ISystemUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit Sensors();

    /// \brief Destructor
    public: virtual ~Sensors();

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override final;

    // Documentation inherited
    public: void Configure(const EntityId &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override final;

    /// \brief Private data pointer.
    private: std::unique_ptr<SensorsPrivate> dataPtr;
  };
  }
}
}
}
#endif