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
#include <iomanip>

#include <ignition/math/Pose3.hh>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/systems/Rendering.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition::gazebo::systems;

using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::systems::RenderingPrivate
{
  /// \brief Query callback for entity that has rendering components.
  /// \param[in] _info Update information.
  /// \param[in] _manager Entity component manager.
  public: void OnUpdate(const UpdateInfo _info,
      EntityComponentManager &_manager);

  public: std::map<EntityId, rendering::VisualPtr> entityMap;

  // TODO(louise) male all these configurable
  /// \brief Engine Name
  public: std::string engineName{"ogre"};

  /// \brief Scene Name
  public: std::string sceneName{"scene"};

  /// \brief Ambient light color
  public: math::Color ambientLight = math::Color(0.3, 0.3, 0.3);

  /// \brief Background color
  public: math::Color backgroundColor = math::Color(0.3, 0.3, 0.8);
};

//////////////////////////////////////////////////
Rendering::Rendering()
  : System(), dataPtr(new RenderingPrivate)
{
  // Render engine
  auto engine = rendering::engine(this->dataPtr->engineName);
  if (!engine)
  {
    ignerr << "Engine [" << this->dataPtr->engineName << "] is not supported"
           << std::endl;
    return;
  }

  // Scene
  auto scene = engine->SceneByName(this->dataPtr->sceneName);
  if (!scene)
  {
    igndbg << "Create scene [" << this->dataPtr->sceneName << "]" << std::endl;
    scene = engine->CreateScene(this->dataPtr->sceneName);
    scene->SetAmbientLight(this->dataPtr->ambientLight);
    scene->SetBackgroundColor(this->dataPtr->backgroundColor);
  }
}

//////////////////////////////////////////////////
Rendering::~Rendering()
{
}

//////////////////////////////////////////////////
void Rendering::Init(std::vector<EntityQueryCallback> &_cbs)
{
  _cbs.push_back(
      std::bind(&RenderingPrivate::OnUpdate, this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
void RenderingPrivate::OnUpdate(const UpdateInfo /*_info*/,
    EntityComponentManager &_manager)
{
  // TODO(louise) Add new visuals to the scen, keeping entity hierarchy
  // TODO(louise) Keep all visual's poses in map, including visuals without geometry
  // TODO(louise) Update visual poses every iteration

//  _manager.Each<components::Name,
//                components::Material,
//                components::Pose>(
//    [&](const EntityId &/*_entity*/,
//        const components::Name *_name,
//        const components::Material *_material,
//        const components::Pose *_pose)
//    {
//      igndbg << "  --  " << _name->Data() << " pose [" << _pose->Data()
//             << "]\n";
//    });
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Rendering,
                    ignition::gazebo::System)

