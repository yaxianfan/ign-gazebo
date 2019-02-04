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

#ifndef IGNITION_GAZEBO_LEVELMANAGER_HH
#define IGNITION_GAZEBO_LEVELMANAGER_HH

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Sensor.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Factory.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/SystemLoader.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // forward declaration
    class SimulationRunner;

    class LevelManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      /// \param[in] _useLevels Whether to use the levels defined. If false, all
      /// entities will be added to the default level
      public: LevelManager(SimulationRunner *_runner, bool _useLevels = false);

      /// \brief Configure the level manager
      public: void Configure();

      /// \brief Load and unload levels
      /// This is where we compute intersections and determine if a performer is
      /// in a level or not. This needs to be called by the simulation runner at
      /// every update cycle
      public: void UpdateLevelsState();

      /// \brief Load entities that have been marked for loading.
      /// \param[in] _namesToLoad List of of entity names to load
      private: void LoadActiveEntities(
          const std::set<std::string> &_namesToLoad);

      /// \brief Unload entities that have been marked for unloading.
      /// \param[in] _namesToUnload List of entity names to unload
      private: void UnloadInactiveEntities(
          const std::set<std::string> &_namesToUnload);

      /// \brief Read level and performer information from the sdf::World
      /// object
      private: void ReadLevelPerformerInfo();

      /// \brief Create performers
      /// Assuming that a simulation runner performer-centered
      private: void CreatePerformers();

      /// \brief Read information about performers from the sdf Element and
      /// create performer entities
      /// \param[in] _sdf sdf::ElementPtr of the ignition::gazebo plugin tag
      private: void ReadPerformers(const sdf::ElementPtr &_sdf);

      /// \brief Read information about levels from the sdf Element and
      /// create level entities
      /// \param[in] _sdf sdf::ElementPtr of the ignition::gazebo plugin tag
      private: void ReadLevels(const sdf::ElementPtr &_sdf);

      /// \brief Determine which entities belong to the default level and
      /// schedule them to be loaded
      private: void ConfigureDefaultLevel();

      /// \brief Determine if a level is active
      /// \param[in] _entity Entity of level to be checked
      /// \return True of the level is currently active
      private: bool IsLevelActive(const Entity _entity) const;

      /// \brief List of currently active levels
      private: std::vector<Entity> activeLevels;

      /// \brief Names of entities to currently active (loaded).
      private: std::set<std::string> activeEntityNames;

      /// \brief Pointer to the simulation runner associated with the level
      /// manager.
      private: SimulationRunner * const runner;

      /// \brief Map of names of references to the containing performer
      private: std::unordered_map<std::string, Entity> performerMap;

      /// \brief Names of all entities that have assigned levels
      private: std::set<std::string> entityNamesInLevels;

      /// \brief Entity of the world
      private: Entity worldEntity = kNullEntity;

      /// \brief Entity of the world
      private: bool useLevels{false};

      /// \brief Entity factory API.
      private: std::unique_ptr<Factory> factory{nullptr};
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_LEVELMANAGER_HH

