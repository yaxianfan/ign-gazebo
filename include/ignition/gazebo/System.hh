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
#ifndef IGNITION_GAZEBO_SYSTEM_HH_
#define IGNITION_GAZEBO_SYSTEM_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \class System System.hh ignition/gazebo/System.hh
    /// \brief Base class for a System.
    ///
    /// A System operates on Entities that have certain Components. A System
    /// will only operate on an Entity if it has all of the required
    /// Components.
    /// \todo(nkoenig) The Callbacks (EntityAdded, EntityRemoved, PreUpdate,
    /// Update, and PostUpdate) should be registered by a system instead of
    /// implicit in the System plugin API.
    class IGNITION_GAZEBO_VISIBLE System
    {
      /// \brief Constructor
      public: System();

      /// \brief Destructor
      public: virtual ~System();
    };

    class IGNITION_GAZEBO_VISIBLE ISystemPreUpdate {
      public: virtual void PreUpdate(const UpdateInfo &_info,
                                     EntityComponentManager &_ecm) = 0;
    };

    class IGNITION_GAZEBO_VISIBLE ISystemUpdate {
      public: virtual void Update(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm) = 0;
    };

    class IGNITION_GAZEBO_VISIBLE ISystemPostUpdate{
      public: virtual void PostUpdate(const UpdateInfo &_info,
                                      const EntityComponentManager &_ecm) = 0;
    };
    }
  }
}
#endif
