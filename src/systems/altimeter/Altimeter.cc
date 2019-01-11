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

#include <ignition/msgs/altimeter.pb.h>

#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "Altimeter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Altimeter sensor class
class ignition::gazebo::systems::AltimeterSensor
{
  /// \brief Constructor
  public: AltimeterSensor();

  /// \brief Destructor
  public: ~AltimeterSensor();

  /// \brief Load the altimeter from an sdf element
  /// \param[in] _sdf SDF element describing the altimeter
  public: void Load(const sdf::ElementPtr &_sdf);

  /// \brief Publish altimeter data over ign transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Vertical position in meters
  public: double verticalPosition = 0.0;

  /// \brief Vertical velocity in meters per second
  public: double verticalVelocity = 0.0;

  /// \brief Vertical reference, i.e. initial sensor position
  public: double verticalReference = 0.0;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief publisher for altimeter data
  public: transport::Node::Publisher pub;
};

/// \brief Private Altimeter data class.
class ignition::gazebo::systems::AltimeterPrivate
{
  /// \brief used to store whether rendering objects have been created.
  public: bool initialized = false;

  /// \brief A map of altimeter entity to its vertical reference
  public: std::unordered_map<Entity, std::unique_ptr<AltimeterSensor>>
      entitySensorMap;

  /// \brief Create altimeter sensor
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateAltimeterEntities(EntityComponentManager &_ecm);

  /// \brief Update altimeter sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateAltimeters(const EntityComponentManager &_ecm);

  /// \brief Helper function to compute world pose of an entity
  /// \param[in] _entity Entity to get the world pose for
  /// \param[in] _ecm Immutable reference to ECM.
  public: math::Pose3d WorldPose(const Entity &_entity,
      const EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
AltimeterSensor::AltimeterSensor()
{
}

//////////////////////////////////////////////////
AltimeterSensor::~AltimeterSensor()
{
}

//////////////////////////////////////////////////
void AltimeterSensor::Load(const sdf::ElementPtr &_sdf)
{
  if (_sdf->HasElement("topic"))
  {
    this->topic = _sdf->Get<std::string>("topic");
    this->pub = this->node.Advertise<ignition::msgs::Altimeter>(this->topic);
  }
}

//////////////////////////////////////////////////
void AltimeterSensor::Publish()
{
  if (!this->pub)
    return;

  msgs::Altimeter msg;
  msg.set_vertical_position(this->verticalPosition);
  msg.set_vertical_velocity(this->verticalVelocity);
  msg.set_vertical_reference(this->verticalReference);
  this->pub.Publish(msg);
}

//////////////////////////////////////////////////
Altimeter::Altimeter() : System(), dataPtr(std::make_unique<AltimeterPrivate>())
{
}

//////////////////////////////////////////////////
Altimeter::~Altimeter()
{
}

//////////////////////////////////////////////////
void Altimeter::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
}

//////////////////////////////////////////////////
void Altimeter::Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    this->dataPtr->CreateAltimeterEntities(_ecm);
    this->dataPtr->initialized = true;
  }

  // Only step if not paused.
  if (!_info.paused)
  {
    this->dataPtr->UpdateAltimeters(_ecm);
  }
}

//////////////////////////////////////////////////
void Altimeter::PostUpdate(const UpdateInfo &_info,
                           const EntityComponentManager &/*_ecm*/)
{
  if (_info.paused)
    return;

  for (auto &it : this->dataPtr->entitySensorMap)
  {
    it.second->Publish();
  }
}

//////////////////////////////////////////////////
void AltimeterPrivate::CreateAltimeterEntities(EntityComponentManager &_ecm)
{
  // Create altimeters
  _ecm.Each<components::Altimeter, components::Pose, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Altimeter *_altimeter,
        const components::Pose * /*_pose*/,
        const components::ParentEntity * /*_parent*/)->bool
      {
        // Get initial pose of parent link and set the reference z pos
        // The WorldPose component was just created and so it's empty
        // We'll compute the world pose manually here
        double verticalReference = this->WorldPose(_entity, _ecm).Pos().Z();
        auto sensor = std::make_unique<AltimeterSensor>();
        sensor->Load(_altimeter->Data());
        sensor->verticalReference = verticalReference;
        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void AltimeterPrivate::UpdateAltimeters(const EntityComponentManager &_ecm)
{
  _ecm.Each<components::Altimeter, components::Pose, components::WorldPose,
            components::WorldLinearVelocity, components::ParentEntity>(
    [&](const Entity &_entity,
        const components::Altimeter * /*_altimeter*/,
        const components::Pose * /*_pose*/,
        const components::WorldPose *_worldPose,
        const components::WorldLinearVelocity *_worldLinearVel,
        const components::ParentEntity * /*_parent*/)->bool
      {
        auto it = this->entitySensorMap.find(_entity);
        if (it != this->entitySensorMap.end())
        {
          math::Vector3d linearVel;
          math::Pose3d worldPose = _worldPose->Data();
          double pos = worldPose.Pos().Z() - it->second->verticalReference;
          double vel = _worldLinearVel->Data().Z();
          it->second->verticalPosition = pos;
          it->second->verticalVelocity = vel;
        }
        else
        {
          ignerr << "Failed to update altimeter: " << _entity << ". "
                 << "Entity not found." << std::endl;
        }

        return true;
      });
}

//////////////////////////////////////////////////
math::Pose3d AltimeterPrivate::WorldPose(const Entity &_entity,
    const EntityComponentManager &_ecm)
{
  // work out pose in world frame
  math::Pose3d worldPose;
  math::Pose3d pose = _ecm.Component<components::Pose>(_entity)->Data();
  auto p = _ecm.Component<components::ParentEntity>(_entity);
  while (p)
  {
    // get pose of parent entity
    auto parentPose =
        _ecm.Component<components::Pose>(p->Data());
    if (!parentPose)
      break;
    // transform pose
    worldPose = pose + parentPose->Data();

    // keep going up the tree
    p = _ecm.Component<components::ParentEntity>(p->Data());
    pose = worldPose;
  }
  return worldPose;
}

IGNITION_ADD_PLUGIN(Altimeter, System,
  Altimeter::ISystemConfigure,
  Altimeter::ISystemUpdate,
  Altimeter::ISystemPostUpdate
)
