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

#include <sdf/Collision.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Factory.hh"

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"

#include "LevelManager.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
LevelManager::LevelManager(SimulationRunner *_runner, const bool _useLevels)
    : runner(_runner), useLevels(_useLevels)
{
  this->factory = std::make_unique<Factory>(
      this->runner->entityCompMgr,
      this->runner->eventMgr);
}

/////////////////////////////////////////////////
void LevelManager::Configure()
{
  this->ReadLevelPerformerInfo();
  this->CreatePerformers();
}

/////////////////////////////////////////////////
void LevelManager::ReadLevelPerformerInfo()
{
  this->worldEntity = this->runner->entityCompMgr.CreateEntity();

  // World components
  this->runner->entityCompMgr.CreateComponent(worldEntity,
                                               components::World());
  this->runner->entityCompMgr.CreateComponent(
      worldEntity, components::Name(this->runner->sdfWorld->Name()));

  auto worldElem = this->runner->sdfWorld->Element();

  // TODO(anyone) This should probaly go somwhere else as it is a global
  // constant.
  const std::string kPluginName{"ignition::gazebo"};

  sdf::ElementPtr pluginElem;
  // Get the ignition::gazebo plugin element
  for (auto plugin = worldElem->GetElement("plugin"); plugin;
       plugin = plugin->GetNextElement("plugin"))
  {
    if (plugin->Get<std::string>("name") == kPluginName)
    {
      pluginElem = plugin;
    }
  }

  if (this->useLevels)
  {
    if (pluginElem == nullptr)
    {
      ignerr << "Could not find a plugin tag with name " << kPluginName << "\n";
      return;
    }
    this->ReadPerformers(pluginElem);
    this->ReadLevels(pluginElem);
  }

  this->ConfigureDefaultLevel();

  // Load world plugins.
  this->runner->EventMgr().Emit<events::LoadPlugins>(this->worldEntity,
      this->runner->sdfWorld->Element());
}


/////////////////////////////////////////////////
void LevelManager::ReadPerformers(const sdf::ElementPtr &_sdf)
{
  IGN_PROFILE("LevelManager::ReadPerformers");

  if (_sdf == nullptr)
    return;

  igndbg << "Reading performer info\n";
  for (auto performer = _sdf->GetElement("performer"); performer;
       performer = performer->GetNextElement("performer"))
  {
    auto name = performer->Get<std::string>("name");

    Entity performerEntity = this->runner->entityCompMgr.CreateEntity();
    // We use the ref to create a parent entity component later on
    std::string ref = performer->GetElement("ref")->GetValue()->GetAsString();
    if (this->performerMap.find(ref) == this->performerMap.end())
    {
      this->performerMap[ref] = performerEntity;
    }
    else
    {
      auto performer2 = this->runner->entityCompMgr.Component<components::Name>(
          this->performerMap[ref]);

      ignerr << "Found multiple performers (" << name << " and "
             << performer2->Data() << ")referring to the same entity\n";
    }

    sdf::Geometry geometry;
    geometry.Load(performer->GetElement("geometry"));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Performer());
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Name(name));
    this->runner->entityCompMgr.CreateComponent(performerEntity,
                                        components::Geometry(geometry));
  }
}

/////////////////////////////////////////////////
void LevelManager::ReadLevels(const sdf::ElementPtr &_sdf)
{
  IGN_PROFILE("LevelManager::ReadLevels");

  igndbg << "Reading levels info\n";

  if (_sdf != nullptr)
  {
    for (auto level = _sdf->GetElement("level"); level;
         level = level->GetNextElement("level"))
    {
      auto name = level->Get<std::string>("name");
      auto pose = level->Get<math::Pose3d>("pose");
      sdf::Geometry geometry;
      geometry.Load(level->GetElement("geometry"));
      std::set<std::string> entityNames;

      for (auto ref = level->GetElement("ref"); ref;
           ref = ref->GetNextElement("ref"))
      {
        std::string entityName = ref->GetValue()->GetAsString();
        // TODO(addisu) Make sure the names are unique
        entityNames.insert(entityName);

        this->entityNamesInLevels.insert(entityName);
      }

      // Entity
      Entity levelEntity = this->runner->entityCompMgr.CreateEntity();

      // Components
      this->runner->entityCompMgr.CreateComponent(
          levelEntity, components::Level());
      this->runner->entityCompMgr.CreateComponent(
          levelEntity, components::Pose(pose));
      this->runner->entityCompMgr.CreateComponent(
          levelEntity, components::Name(name));
      this->runner->entityCompMgr.CreateComponent(
          levelEntity, components::ParentEntity(worldEntity));
      this->runner->entityCompMgr.CreateComponent(
          levelEntity, components::LevelEntityNames(entityNames));
      this->runner->entityCompMgr.CreateComponent(
          levelEntity, components::Geometry(geometry));
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::ConfigureDefaultLevel()
{
  IGN_PROFILE("LevelManager::ConfigureDefaultLevel");

  // Create the default level. This level contains all entities not contained by
  // any other level.
  Entity defaultLevel = this->runner->entityCompMgr.CreateEntity();

  // Go through all entities in the world and find ones not in the
  // set entityNamesInLevels

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    // If model is a performer, it will be handled separately
    if (this->performerMap.find(model->Name()) != this->performerMap.end())
    {
      continue;
    }

    if (this->entityNamesInLevels.find(model->Name()) ==
        this->entityNamesInLevels.end())
    {
      this->entityNamesInDefault.insert(model->Name());
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (this->entityNamesInLevels.find(light->Name()) ==
        this->entityNamesInLevels.end())
    {
      this->entityNamesInDefault.insert(light->Name());
    }
  }
  // Components
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::Level());
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::ParentEntity(this->worldEntity));
  this->runner->entityCompMgr.CreateComponent(
      defaultLevel, components::LevelEntityNames(this->entityNamesInDefault));

  // Add default level to levels to load
  this->entityNamesToLoad.insert(this->entityNamesInDefault.begin(),
                                 this->entityNamesInDefault.end());
}

/////////////////////////////////////////////////
void LevelManager::CreatePerformers()
{
  IGN_PROFILE("LevelManager::CreatePerformers");

  if (this->worldEntity == kNullEntity)
  {
    ignerr << "Could not find the world entity while creating performers\n";
    return;
  }
  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    if (this->performerMap.find(model->Name()) != this->performerMap.end() )
    {
      Entity modelEntity = this->factory->CreateEntities(model);

      // Create a component on the performer entity that points to this model
      this->runner->entityCompMgr.SetParentEntity(
          this->performerMap[model->Name()], modelEntity);
      this->runner->entityCompMgr.CreateComponent(
          this->performerMap[model->Name()],
          components::ParentEntity(modelEntity));

      // Add parent world to the model
      this->runner->entityCompMgr.SetParentEntity(modelEntity, this->worldEntity);
      this->runner->entityCompMgr.CreateComponent(
          modelEntity, components::ParentEntity(this->worldEntity));
    }
  }
}

/////////////////////////////////////////////////
void LevelManager::UpdateLevelsState()
{
  IGN_PROFILE("LevelManager::UpdateLevelsState");

  this->runner->entityCompMgr.Each<components::Performer, components::Geometry,
                                   components::ParentEntity>(
      [&](const Entity &, const components::Performer *,
          const components::Geometry *_geometry,
          const components::ParentEntity *_parent) -> bool
      {
        auto pose = this->runner->entityCompMgr.Component<components::Pose>(
            _parent->Data());
        // We assume the geometry contains a box.
        auto perfBox = _geometry->Data().BoxShape();

        math::AxisAlignedBox performerVolume{
             pose->Data().Pos() - perfBox->Size() / 2,
             pose->Data().Pos() + perfBox->Size() / 2};


        // loop through levels and check for intersections
        this->runner->entityCompMgr.Each<components::Level, components::Pose,
                                         components::Geometry,
                                         components::LevelEntityNames>(
            [&](const Entity &, const components::Level *,
                const components::Pose *_pose,
                const components::Geometry *_levelGeometry,
                const components::LevelEntityNames *_entityNames) -> bool
            {
              // Check if the performer is in this level
              // assume a box for now
              auto box = _levelGeometry->Data().BoxShape();
              auto center = _pose->Data().Pos();
              math::AxisAlignedBox region{center - box->Size() / 2,
                                          center + box->Size() / 2};

              if (region.Intersects(performerVolume))
              {
                this->entityNamesToLoad.insert(_entityNames->Data().begin(),
                                               _entityNames->Data().end());
              }
              else
              {
                // mark the entity to be unloaded if it's currently active
                std::copy_if(_entityNames->Data().begin(),
                             _entityNames->Data().end(),
                             std::inserter(this->entityNamesToUnload,
                                           this->entityNamesToUnload.begin()),
                             [this](const std::string &_name) -> bool
                             {
                               return this->activeEntityNames.find(_name) !=
                                      this->activeEntityNames.end();
                             });
              }
              return true;
            });

        return true;
      });

  // Filter the entityNamesToUnload so that if a level is marked to be loaded by
  // one performer check and marked to be unloaded by another, we don't end up
  // unloading it
  std::set<std::string> tmpToUnload;
  std::set_difference(
      this->entityNamesToUnload.begin(), this->entityNamesToUnload.end(),
      this->entityNamesToLoad.begin(), this->entityNamesToLoad.end(),
      std::inserter(tmpToUnload, tmpToUnload.begin()));
  this->entityNamesToUnload = std::move(tmpToUnload);

  // Filter out the entities that are already active
  std::set<std::string> tmpToLoad;
  std::set_difference(
      this->entityNamesToLoad.begin(), this->entityNamesToLoad.end(),
      this->activeEntityNames.begin(), this->activeEntityNames.end(),
      std::inserter(tmpToLoad, tmpToLoad.begin()));
  this->entityNamesToLoad = std::move(tmpToLoad);

  // ---------------------- DEBUG ---------------------
  static std::size_t counter = 0;

  if (this->entityNamesToLoad.size() > 0)
  {
    std::stringstream ss;
    ss << counter << ": Levels to load:";
    std::copy(this->entityNamesToLoad.begin(), this->entityNamesToLoad.end(),
              std::ostream_iterator<std::string>(ss, " "));
    igndbg << ss.str() << std::endl;
  }

  if (this->entityNamesToUnload.size() > 0)
  {
    std::stringstream ss;
    ss << counter << ": Levels to unload:";
    std::copy(this->entityNamesToUnload.begin(),
              this->entityNamesToUnload.end(),
              std::ostream_iterator<std::string>(ss, " "));
    igndbg << ss.str() << std::endl;
  }
  ++counter;
  // ---------------------- END DEBUG ---------------------
}

/////////////////////////////////////////////////
void LevelManager::LoadActiveLevels()
{
  IGN_PROFILE("LevelManager::LoadActiveLevels");

  if (this->worldEntity == kNullEntity)
  {
    ignerr << "Could not find the world entity while loading levels\n";
    return;
  }

  // Models
  for (uint64_t modelIndex = 0;
       modelIndex < this->runner->sdfWorld->ModelCount(); ++modelIndex)
  {
    // There is no sdf::World::ModelByName so we have to iterate by index and
    // check if the model is in this level
    auto model = this->runner->sdfWorld->ModelByIndex(modelIndex);
    if (this->entityNamesToLoad.find(model->Name()) !=
        this->entityNamesToLoad.end())
    {
      Entity modelEntity = this->factory->CreateEntities(model);

      this->runner->entityCompMgr.SetParentEntity(modelEntity,
          this->worldEntity);
      this->runner->entityCompMgr.CreateComponent(
          modelEntity, components::ParentEntity(this->worldEntity));
    }
  }

  // Lights
  for (uint64_t lightIndex = 0;
       lightIndex < this->runner->sdfWorld->LightCount(); ++lightIndex)
  {
    auto light = this->runner->sdfWorld->LightByIndex(lightIndex);
    if (this->entityNamesToLoad.find(light->Name()) !=
        this->entityNamesToLoad.end())
    {
      Entity lightEntity = this->factory->CreateEntities(light);

      this->runner->entityCompMgr.SetParentEntity(lightEntity,
          this->worldEntity);
      this->runner->entityCompMgr.CreateComponent(
          lightEntity, components::ParentEntity(this->worldEntity));
    }
  }

  this->activeEntityNames.insert(this->entityNamesToLoad.begin(),
                                 this->entityNamesToLoad.end());
  this->entityNamesToLoad.clear();
}

/////////////////////////////////////////////////
void LevelManager::UnloadInactiveLevels()
{
  IGN_PROFILE("LevelManager::UnloadInactiveLevels");

  this->runner->entityCompMgr.Each<components::Model, components::Name>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_name) -> bool
      {
        if (this->entityNamesToUnload.find(_name->Data()) !=
            this->entityNamesToUnload.end())
        {
          this->runner->entityCompMgr.RequestEraseEntity(_entity);
        }
        return true;
      });
  for (const auto &_name : this->entityNamesToUnload)
  {
    this->activeEntityNames.erase(_name);
  }
  this->entityNamesToUnload.clear();
}

