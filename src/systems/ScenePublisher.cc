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

#include <ignition/math/graph/Graph.hh>
#include <ignition/msgs/scene.pb.h>
#include <ignition/plugin/RegisterMore.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/systems/ScenePublisher.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

//////////////////////////////////////////////////
void AddVisuals(msgs::Link *_msg,
    const EntityId _id,
    const math::graph::DirectedGraph<google::protobuf::Message *, bool>
        &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto visualMsg = dynamic_cast<msgs::Visual *>(vertex.second.get().Data());
    if (!visualMsg)
      continue;

    _msg->add_visual()->CopyFrom(*visualMsg);
  }
}

//////////////////////////////////////////////////
void AddLinks(msgs::Model *_msg,
    const EntityId _id,
    const math::graph::DirectedGraph<google::protobuf::Message *, bool>
        &_graph)
{
  if (!_msg)
    return;

  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto linkMsg = dynamic_cast<msgs::Link *>(vertex.second.get().Data());
    if (!linkMsg)
      continue;

    // Visuals
    AddVisuals(linkMsg, vertex.second.get().Id(), _graph);

    _msg->add_link()->CopyFrom(*linkMsg);
  }
}

//////////////////////////////////////////////////
template<typename T>
void AddModels(T _msg,
    const EntityId _id,
    const math::graph::DirectedGraph<google::protobuf::Message *, bool>
        &_graph)
{
  for (const auto &vertex : _graph.AdjacentsFrom(_id))
  {
    auto modelMsg = dynamic_cast<msgs::Model *>(vertex.second.get().Data());
    if (!modelMsg)
      continue;

    // Nested models
    AddModels(modelMsg, vertex.second.get().Id(), _graph);

    // Links
    AddLinks(modelMsg, vertex.second.get().Id(), _graph);

    _msg->add_model()->CopyFrom(*modelMsg);
  }
}

// Private data class.
class ignition::gazebo::systems::ScenePublisherPrivate
{
  /// \brief Query callback for entity that has physics components.
  /// \param[in] _info Update information.
  /// \param[in] _manager Entity component manager.
  public: void OnUpdate(const UpdateInfo _info,
      EntityComponentManager &_manager);

  public: transport::Node node;

  public: transport::Node::Publisher scenePub;
};

//////////////////////////////////////////////////
ScenePublisher::ScenePublisher()
  : System(), dataPtr(std::make_unique<ScenePublisherPrivate>())
{
  // TODO(louise) Make topic configurable
  this->dataPtr->scenePub =
      this->dataPtr->node.Advertise<msgs::Scene>("/scene");
}

//////////////////////////////////////////////////
ScenePublisher::~ScenePublisher()
{
}

//////////////////////////////////////////////////
void ScenePublisher::Init(std::vector<EntityQueryCallback> &_cbs)
{
  _cbs.push_back(
      std::bind(&ScenePublisherPrivate::OnUpdate, this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
void ScenePublisherPrivate::OnUpdate(const UpdateInfo /*_info*/,
    EntityComponentManager &_manager)
{
  // TODO(louise) Get <scene> from SDF
  // TODO(louise) Fill message header

  // First populate a graph, then populate a message from the graph
  math::graph::DirectedGraph<google::protobuf::Message *, bool> graph;

  // World
  EntityId worldId = kNullEntity;
  _manager.Each<components::World,
                components::Name>(
    [&graph, &_manager, &worldId](const EntityId &_entity,
        const components::World */*_worldComp*/,
        const components::Name *_nameComp)
    {
      // TODO(louise) Support multiple worlds
      if (kNullEntity != worldId)
      {
        ignerr << "Skipping world [" << _nameComp->Data() << "]" << std::endl;
        return;
      }
      worldId = _entity;

      graph.AddVertex(_nameComp->Data(), nullptr, _entity);
    });

  if (kNullEntity == worldId)
  {
    ignerr << "Failed to find world entity" << std::endl;
    return;
  }

  // Models
  _manager.Each<components::Model,
                components::Name,
                components::ParentEntity,
                components::Pose>(
    [&graph, &_manager](const EntityId &_entity,
        const components::Model */*_modelComp*/,
        const components::Name *_nameComp,
        const components::ParentEntity *_parentComp,
        const components::Pose *_poseComp)
    {
      auto modelMsg = new msgs::Model();
      modelMsg->set_id(_entity);
      modelMsg->set_name(_nameComp->Data());
      modelMsg->mutable_pose()->CopyFrom(msgs::Convert(
          _poseComp->Data()));

      graph.AddVertex(_nameComp->Data(), modelMsg, _entity);
      graph.AddEdge({_parentComp->Id(), _entity}, true);
    });

  // Links
  _manager.Each<components::Link,
                components::Name,
                components::ParentEntity,
                components::Pose>(
    [&graph, &_manager](const EntityId &_entity,
        const components::Link */*_linkComp*/,
        const components::Name *_nameComp,
        const components::ParentEntity *_parentComp,
        const components::Pose *_poseComp)
    {
      auto linkMsg = new msgs::Link();
      linkMsg->set_id(_entity);
      linkMsg->set_name(_nameComp->Data());
      linkMsg->mutable_pose()->CopyFrom(msgs::Convert(
          _poseComp->Data()));

      graph.AddVertex(_nameComp->Data(), linkMsg, _entity);
      graph.AddEdge({_parentComp->Id(), _entity}, true);
    });

  // Visuals
  _manager.Each<components::Visual,
                components::Geometry,
                components::Material,
                components::Name,
                components::ParentEntity,
                components::Pose>(
    [&graph, &_manager](const EntityId &_entity,
        const components::Visual */*_visualComp*/,
        const components::Geometry *_geometryComp,
        const components::Material *_materialComp,
        const components::Name *_nameComp,
        const components::ParentEntity *_parentComp,
        const components::Pose *_poseComp)
    {
      auto visualMsg = new msgs::Visual();
      visualMsg->set_id(_entity);
      visualMsg->set_parent_id(_parentComp->Id());
      visualMsg->set_name(_nameComp->Data());
      visualMsg->mutable_pose()->CopyFrom(msgs::Convert(
          _poseComp->Data()));

      visualMsg->mutable_geometry()->CopyFrom(
          Convert<msgs::Geometry>(_geometryComp->Data()));

      visualMsg->mutable_material()->CopyFrom(
          Convert<msgs::Material>(_materialComp->Data()));

      graph.AddVertex(_nameComp->Data(), visualMsg, _entity);
      graph.AddEdge({_parentComp->Id(), _entity}, true);
    });

  // Populate scene message
  msgs::Scene sceneMsg;
  AddModels(&sceneMsg, worldId, graph);

  // Publish
  this->scenePub.Publish(sceneMsg);
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::ScenePublisher,
                    ignition::gazebo::System)

