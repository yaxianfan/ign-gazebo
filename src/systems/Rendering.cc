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
#include <sdf/Box.hh>

#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/systems/Rendering.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
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

  /// \brief Maps entities to visuals.
  public: std::map<EntityId, rendering::VisualPtr> entityMap;

  /// \brief Keep pointer to the scene.
  public: rendering::ScenePtr scene;

  // TODO(louise) male all these configurable
  /// \brief Engine Name
  public: std::string engineName{"ogre"};

  /// \brief Scene Name
  public: std::string sceneName{"scene"};

  /// \brief Ambient light color
  public: math::Color ambientLight = math::Color(0.3, 0.3, 0.3);

  /// \brief Background color
  public: math::Color backgroundColor = math::Color(0.7, 0.7, 0.9);
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
  this->dataPtr->scene = engine->SceneByName(this->dataPtr->sceneName);
  if (!this->dataPtr->scene)
  {
    igndbg << "Create scene [" << this->dataPtr->sceneName << "]" << std::endl;
    this->dataPtr->scene = engine->CreateScene(this->dataPtr->sceneName);
    this->dataPtr->scene->SetAmbientLight(this->dataPtr->ambientLight);
    this->dataPtr->scene->SetBackgroundColor(this->dataPtr->backgroundColor);
  }
  else
  {
    ignwarn << "Scene [" << this->dataPtr->sceneName << "] created before rendering system."
            << std::endl;
  }

  // Debug: why is this box not showing?!
  // Is the scene ever being repainted? :(
  auto green = this->dataPtr->scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  auto box = this->dataPtr->scene->CreateVisual();
  box->AddGeometry(this->dataPtr->scene->CreateBox());
  box->SetMaterial(green);
  this->dataPtr->scene->RootVisual()->AddChild(box);
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
  // Everything which has a pose becomes a node in the scene graph (i.e. a rendering::Visual),
  // even if it won't have a geometry, because their pose may affect children.
  _manager.Each<components::ParentEntity,
                components::Pose>(
    [this, &_manager](const EntityId &_entity,
        const components::ParentEntity *_parentComp,
        const components::Pose *_poseComp)
    {
      auto vis = this->entityMap[_entity];

      // Visual not created yet, create it
      if (!vis)
      {
        vis = this->scene->CreateVisual();
      }

      // Set pose
      vis->SetLocalPosition(_poseComp->Data().Pos());
      vis->SetLocalRotation(_poseComp->Data().Rot());

      // Only pose can be updated after creation, so shortcut here if visual has
      // already been created
      if (nullptr != this->entityMap[_entity])
      {
        return;
      }

      // Add to parent
      // TODO(louise) don't assume parent was created before child
      auto parentVis = this->entityMap[_parentComp->Id()];
      if (!parentVis)
      {
        parentVis = this->scene->RootVisual();
      }
      parentVis->AddChild(vis);

      // Geometry
      if (auto geomComp = _manager.Component<components::Geometry>(_entity))
      {
        if (geomComp->Data().Type() == sdf::GeometryType::BOX && geomComp->Data().BoxShape())
        {
          auto box = this->scene->CreateBox();
          vis->AddGeometry(box);

          auto boxSdf = geomComp->Data().BoxShape();
          vis->SetLocalScale(boxSdf->Size());
        }
        else
        {
          ignerr << "Geometry type [" << static_cast<int>(geomComp->Data().Type())
                 << "] not supported" << std::endl;
        }
      }

      // Material
      if (auto matComp = _manager.Component<components::Material>(_entity))
      {
        auto mat = this->scene->CreateMaterial();
        mat->SetAmbient(matComp->Data().Ambient());
        mat->SetDiffuse(matComp->Data().Diffuse());
        mat->SetSpecular(matComp->Data().Specular());

        vis->SetMaterial(mat);
      }

      this->entityMap[_entity] = vis;
    });
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Rendering,
                    ignition::gazebo::System)

