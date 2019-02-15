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

#include <unordered_map>

#include <ignition/msgs/contact.pb.h>
#include <ignition/msgs/contacts.pb.h>

#include <ignition/plugin/Register.hh>

#include <sdf/Element.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Contact.hh"
#include "ignition/gazebo/components/ContactData.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

#include "Contact.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ContactSensor
{
  /// \brief Load the Contact sensor from an sdf element
  /// \param[in] _sdf SDF element describing the Contact sensor
  /// \param[in] _topic string with topic name
  /// \param[in] _collisionEntities A list of entities that act as contact
  /// sensors
  public: void Load(const sdf::ElementPtr &_sdf, std::string _topic,
                    std::vector<Entity> _collisionEntities);

  /// \brief Add contacts to the list to be published
  /// \param[in] _stamp Time stamp of the sensor measurement
  /// \param[in] _contacts A contact message to be added to the list
  public: void AddContacts(const std::chrono::steady_clock::duration &_stamp,
                           const msgs::Contacts &_contacts);

  /// \brief Publish sensor data over ign transport
  public: void Publish();

  /// \brief Topic to publish data to
  public: std::string topic;

  /// \brief Message to publish
  public: msgs::Contacts contactsMsg;

  /// \brief Ign transport node
  public: transport::Node node;

  /// \brief Ign transport publisher
  public: transport::Node::Publisher pub;

  /// \brief Entities for which this sensor publishes data
  public: std::vector<Entity> collisionEntities;
};

class ignition::gazebo::systems::ContactPrivate
{
  /// \brief Create sensors that correspond to entities in the simulation
  /// \param[in] _ecm Mutable reference to ECM.
  public: void CreateSensors(EntityComponentManager &_ecm);

  /// \brief Update and publish sensor data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateSensors(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm);

  /// \brief Remove sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveSensors(const EntityComponentManager &_ecm);

  /// \brief A map of Contact entity to its Contact sensor.
  public: std::unordered_map<Entity,
      std::unique_ptr<ContactSensor>> entitySensorMap;

  /// \brief The world entity this system is attached to
  public: Entity worldEntity = kNullEntity;
};

//////////////////////////////////////////////////
void ContactSensor::Load(const sdf::ElementPtr &_sdf, std::string _topic,
                         std::vector<Entity> _collisionEntities)
{
  this->collisionEntities = std::move(_collisionEntities);

  auto contactElem = _sdf->GetElement("contact");
  if (contactElem ->HasElement("topic"))
  {
    this->topic = contactElem ->Get<std::string>("topic");
  }
  else
  {
    // use default topic for sensor
    this->topic = std::move(_topic);
  }

  igndbg << "Contact system publishing on " << this->topic << std::endl;
  this->pub = this->node.Advertise<ignition::msgs::Contacts>(this->topic);
}

//////////////////////////////////////////////////
void ContactSensor::AddContacts(
    const std::chrono::steady_clock::duration &_stamp,
    const msgs::Contacts &_contacts)
{
  auto ts = math::durationToSecNsec(_stamp);
  for (const auto &contact : _contacts.contact())
  {
    auto *newContact = this->contactsMsg.add_contact();
    newContact->CopyFrom(contact);
    newContact->mutable_header()->mutable_stamp()->set_sec(ts.first);
    newContact->mutable_header()->mutable_stamp()->set_nsec(ts.second);
  }

  this->contactsMsg.mutable_header()->mutable_stamp()->set_sec(ts.first);
  this->contactsMsg.mutable_header()->mutable_stamp()->set_nsec(ts.second);
}

//////////////////////////////////////////////////
void ContactSensor::Publish()
{
  // Only publish if there are contacts
  if (this->contactsMsg.contact_size() > 0)
  {
    this->pub.Publish(this->contactsMsg);
    this->contactsMsg.Clear();
  }
}

//////////////////////////////////////////////////
void ContactPrivate::CreateSensors(EntityComponentManager &_ecm)
{
  _ecm.EachNew<components::Contact>(
      [&](const Entity &_entity, const components::Contact *_contact) -> bool
      {
        // Check if the parent entity is a link
        auto *parentEntity = _ecm.Component<components::ParentEntity>(_entity);
        if (nullptr == parentEntity)
          return true;

        auto *linkComp = _ecm.Component<components::Link>(parentEntity->Data());
        if (nullptr == linkComp)
        {
          // Contact sensors should only be attached to links
          return true;
        }

        auto collisionElem =
            _contact->Data()->GetElement("contact")->GetElement("collision");

        std::vector<Entity> collisionEntities;
        // Get all the collision elements
        for (; collisionElem;
             collisionElem = collisionElem->GetNextElement("collision"))
        {
          auto collisionName = collisionElem->Get<std::string>();
          // Get collision entity that matches the name given by the sensor's
          // configuration.
          auto childEntities = _ecm.ChildrenByComponents(
              parentEntity->Data(), components::Collision(),
              components::Name(collisionName));

          if (!childEntities.empty())
          {
            // We assume that if childEntities is not empty, it only has one
            // element.
            collisionEntities.push_back(childEntities.front());

            // Create component to be filled by physics.
            _ecm.CreateComponent(childEntities.front(),
                                 components::ContactData());
          }
        }

        std::string defaultTopic = scopedName(_entity, _ecm, "/") + "/contact";

        auto sensor = std::make_unique<ContactSensor>();
        sensor->Load(_contact->Data(), defaultTopic, collisionEntities);
        this->entitySensorMap.insert(
            std::make_pair(_entity, std::move(sensor)));

        return true;
      });
}

//////////////////////////////////////////////////
void ContactPrivate::UpdateSensors(const UpdateInfo &_info,
                                   const EntityComponentManager &_ecm)
{
  for (const auto &item : this->entitySensorMap)
  {
    for (const Entity &entity : item.second->collisionEntities)
    {
      auto contacts = _ecm.Component<components::ContactData>(entity);

      // We will assume that the ContactData component will have been created if
      // this entity is in the collisionEntities list
      if (contacts->Data().contact_size() > 0)
      {
        item.second->AddContacts(_info.simTime, contacts->Data());
      }
    }
  }
}

//////////////////////////////////////////////////
void ContactPrivate::RemoveSensors(
    const EntityComponentManager &_ecm)
{
  _ecm.EachRemoved<components::Contact>(
    [&](const Entity &_entity,
        const components::Contact *)->bool
      {
        auto sensorId = this->entitySensorMap.find(_entity);
        if (sensorId == this->entitySensorMap.end())
        {
          ignerr << "Internal error, missing Contact sensor for entity ["
                 << _entity << "]" << std::endl;
          return true;
        }

        this->entitySensorMap.erase(sensorId);

        return true;
      });
}
//////////////////////////////////////////////////
Contact::Contact() : System(), dataPtr(std::make_unique<ContactPrivate>())
{
}

void Contact::Configure(const Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &,
                        EntityComponentManager &, EventManager &)
{
  this->dataPtr->worldEntity = _entity;
}

//////////////////////////////////////////////////
void Contact::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm)
{
  this->dataPtr->CreateSensors(_ecm);
}

//////////////////////////////////////////////////
void Contact::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  if (!_info.paused)
  {
    this->dataPtr->UpdateSensors(_info, _ecm);

    for (auto &it : this->dataPtr->entitySensorMap)
    {
      // Publish sensor data
      it.second->Publish();
    }
  }

  this->dataPtr->RemoveSensors(_ecm);
}

IGNITION_ADD_PLUGIN(Contact, System,
  Contact::ISystemConfigure,
  Contact::ISystemPreUpdate,
  Contact::ISystemPostUpdate
)

