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
#ifndef IGNITION_GAZEBO_EVENTS_HH_
#define IGNITION_GAZEBO_EVENTS_HH_

#include <sdf/Element.hh>

#include <ignition/common/Event.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \brief Namespace for all events. Refer to the EventManager class for
    /// more information about events.
    namespace events
    {
      /// \brief The pause event can be used to pause or unpause simulation.
      /// Emit a value of true to pause simulation, and emit a value of false
      /// to unpause simulation.
      ///
      /// For example, to pause simulation use:
      /// \code
      /// eventManager.Emit<ignition::gazebo::events::Pause>(true);
      /// \endcode
      using Pause = ignition::common::EventT<void(bool), struct PauseTag>;

      /// \brief Event used to load plugins for an entity into simulation.
      /// Pass in the entity which will own the plugins, and an SDF element for
      /// the entity, which may contain multiple <plugin> tags.
      using LoadPlugins = common::EventT<void(Entity, sdf::ElementPtr),
          struct LoadPluginsTag>;
      }
    }  // namespace events
  }  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_EVENTS_HH_
