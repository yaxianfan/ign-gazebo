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

#ifndef IGNITION_GAZEBO_SYSTEMS_STATE_PUBLISHER_HH_
#define IGNITION_GAZEBO_SYSTEMS_STATE_PUBLISHER_HH_

#include <memory>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  /// \brief The JointStatePub system publishes joint state information.
  ///
  /// The following parameters are used by the system:
  ///
  class IGNITION_GAZEBO_VISIBLE StatePublisher
      : public System,
        public ISystemConfigure,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: StatePublisher();

    /// \brief Destructor
    public: ~StatePublisher() override = default;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &,
                           EntityComponentManager &_ecm, EventManager &);

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    private: Model model;
    private: transport::Node node;
    private: std::unique_ptr<transport::Node::Publisher> modelPub;
  };
  }
}
}
}

#endif
