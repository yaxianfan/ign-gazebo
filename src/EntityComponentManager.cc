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
#include "ignition/gazebo/EntityComponentManager.hh"

#include <any>
#include <map>
#include <set>
#include <vector>

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::EntityComponentManagerPrivate
{
  /// \brief Map of component storage classes. The key is a component
  /// type id, and the value is a pointer to the component storage.
  public: std::map<ComponentTypeId,
          std::unique_ptr<ComponentStorageBase>> components;

  /// \brief Map of component update requests. The key is a pair of entity id
  /// and component type id, and the value is a pair of update priority (size_t)
  /// and the value of the component.
  /// TODO(anyone) replace with a lock free data structure
  public: std::map<std::pair<EntityId, ComponentTypeId>,
                   std::pair<size_t, std::any>> componentUpdateRequests;

  /// \brief Instances of entities
  public: std::vector<Entity> entities;

  /// \brief Deleted entity ids that can be reused
  public: std::set<EntityId> availableEntityIds;

  /// \brief Entities that need to be erased.
  public: std::set<EntityId> toEraseEntityIds;

  /// \brief Flag that indicates if all entities should be erased.
  public: bool eraseAllEntities{false};

  /// \brief The set of components that each entity has
  public: std::map<EntityId, std::vector<ComponentKey>> entityComponents;

  /// \brief The set of all views.
  public: mutable std::map<ComponentTypeKey, View> views;
};

//////////////////////////////////////////////////
EntityComponentManager::EntityComponentManager()
  : dataPtr(new EntityComponentManagerPrivate)
{
}

//////////////////////////////////////////////////
EntityComponentManager::~EntityComponentManager()
{
}

//////////////////////////////////////////////////
size_t EntityComponentManager::EntityCount() const
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  return this->dataPtr->entities.size() -
    this->dataPtr->availableEntityIds.size();
}

//////////////////////////////////////////////////
EntityId EntityComponentManager::CreateEntity()
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  EntityId id = kNullEntity;

  if (!this->dataPtr->availableEntityIds.empty())
  {
    // Reuse the smallest available EntityId
    id = *(this->dataPtr->availableEntityIds.begin());
    this->dataPtr->availableEntityIds.erase(
        this->dataPtr->availableEntityIds.begin());
    this->dataPtr->entities[id] = std::move(Entity(id));
  }
  else
  {
    // Create a brand new Id
    id = this->dataPtr->entities.size();
    this->dataPtr->entities.push_back(std::move(Entity(id)));
  }

  return id;
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestEraseEntity(EntityId _id)
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  this->dataPtr->toEraseEntityIds.insert(_id);
}

/////////////////////////////////////////////////
void EntityComponentManager::RequestEraseEntities()
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  this->dataPtr->eraseAllEntities = true;
}

/////////////////////////////////////////////////
void EntityComponentManager::ProcessEraseEntityRequests()
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  // Short-cut if erasing all entities
  if (this->dataPtr->eraseAllEntities)
  {
    this->dataPtr->eraseAllEntities = false;
    this->dataPtr->entities.clear();
    this->dataPtr->entityComponents.clear();
    this->dataPtr->availableEntityIds.clear();
    this->dataPtr->toEraseEntityIds.clear();

    for (std::pair<const ComponentTypeId,
        std::unique_ptr<ComponentStorageBase>> &comp: this->dataPtr->components)
    {
      comp.second->RemoveAll();
    }

    // All views are now invalid.
    this->dataPtr->views.clear();
  }
  else
  {
    // Otherwise iterate through the list of entities to erase.
    for (const EntityId _id : this->dataPtr->toEraseEntityIds)
    {
      // Make sure the entity exists and is not erased.
      if (!this->HasEntityImpl(_id))
        continue;

      // Insert the entity into the set of available ids.
      this->dataPtr->availableEntityIds.insert(_id);

      // Remove the components, if any.
      if (this->dataPtr->entityComponents.find(_id) !=
          this->dataPtr->entityComponents.end())
      {
        for (const ComponentKey &_key : this->dataPtr->entityComponents.at(_id))
          this->dataPtr->components.at(_key.first)->Remove(_key.second);

        // Remove the entry in the entityComponent map
        this->dataPtr->entityComponents.erase(_id);
      }

      // Remove the entity from views.
      for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
      {
        view.second.EraseEntity(_id, view.first);
      }
    }
    // Clear the set of entities to erase.
    this->dataPtr->toEraseEntityIds.clear();
  }
}

/////////////////////////////////////////////////
bool EntityComponentManager::RemoveComponent(
    const EntityId _id, const ComponentKey &_key)
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  // Make sure the entity exists and has the component.
  if (!this->EntityHasComponentImpl(_id, _key))
    return false;

  auto entityComponentIter = std::find(
      this->dataPtr->entityComponents[_id].begin(),
      this->dataPtr->entityComponents[_id].end(), _key);

  this->dataPtr->components.at(_key.first)->Remove(_key.second);
  this->dataPtr->entityComponents[_id].erase(entityComponentIter);

  this->UpdateViews(_id);
  return true;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponent(const EntityId _id,
    const ComponentKey &_key) const
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  return this->EntityHasComponentImpl(_id, _key);
}

//////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentImpl(const EntityId _id,
    const ComponentKey &_key) const
{
  if (this->HasEntityImpl(_id))
  {
    auto compIter = this->dataPtr->entityComponents.find(_id);
    if (compIter != this->dataPtr->entityComponents.end())
    {
      return std::find(compIter->second.begin(), compIter->second.end(),
                       _key) != compIter->second.end();
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentType(const EntityId _id,
    const ComponentTypeId &_typeId) const
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  return this->EntityHasComponentTypeImpl(_id, _typeId);
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityHasComponentTypeImpl(const EntityId _id,
    const ComponentTypeId &_typeId) const
{
  if (!this->HasEntityImpl(_id))
    return false;

  std::map<EntityId, std::vector<ComponentKey>>::const_iterator iter =
    this->dataPtr->entityComponents.find(_id);

  if (iter == this->dataPtr->entityComponents.end())
    return false;

  return std::find_if(iter->second.begin(), iter->second.end(),
      [&] (const ComponentKey &_key)
      {
        return _key.first == _typeId;
      }) != iter->second.end();
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasEntity(const EntityId _id) const
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  return this->HasEntityImpl(_id);
}

//////////////////////////////////////////////////
bool EntityComponentManager::HasEntityImpl(EntityId _id) const
{
  return
    // Check that the _id is in range
    _id >= 0 && _id < static_cast<EntityId>(this->dataPtr->entities.size())
    // Check that the _id is not deleted (not in the available entity set)
    && this->dataPtr->availableEntityIds.find(_id) ==
       this->dataPtr->availableEntityIds.end();
}

/////////////////////////////////////////////////
ComponentKey EntityComponentManager::CreateComponentImplementation(
    const EntityId _entityId, const ComponentTypeId _componentTypeId,
    const void *_data)
{
  // Instantiate the new component.
  std::pair<ComponentId, bool> componentIdPair =
    this->dataPtr->components[_componentTypeId]->Create(_data);

  ComponentKey componentKey{_componentTypeId, componentIdPair.first};

  this->dataPtr->entityComponents[_entityId].push_back(componentKey);

  if (componentIdPair.second)
    this->RebuildViewsImpl();
  else
    this->UpdateViews(_entityId);

  return componentKey;
}


/////////////////////////////////////////////////
bool EntityComponentManager::EntityMatches(EntityId _id,
    const std::set<ComponentTypeId> &_types) const
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  return this->EntityMatchesImpl(_id, _types);
}

/////////////////////////////////////////////////
bool EntityComponentManager::EntityMatchesImpl(EntityId _id,
    const std::set<ComponentTypeId> &_types) const
{
  std::map<EntityId, std::vector<ComponentKey>>::const_iterator iter =
    this->dataPtr->entityComponents.find(_id);
  if (iter == this->dataPtr->entityComponents.end())
    return false;

  // \todo(nkoenig) The performance of this could be improved. Ideally we
  // wouldn't need two loops to confirm that an entity matches a set of
  // types. It might be possible to create bitmask for component sets.
  // Fixing this might not be high priority, unless we expect frequent
  // creation of entities and/or queries.
  for (const ComponentTypeId &type : _types)
  {
    bool found = false;
    for (const ComponentKey &comp : iter->second)
    {
      if (comp.first == type)
      {
        found = true;
        break;
      }
    }
    if (!found)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
ComponentId EntityComponentManager::EntityComponentIdFromType(
    const EntityId _id, const ComponentTypeId _type) const
{
  return this->EntityComponentIdFromTypeImpl(_id, _type);
}

/////////////////////////////////////////////////
ComponentId EntityComponentManager::EntityComponentIdFromTypeImpl(
    const EntityId _id, const ComponentTypeId _type) const
{
  std::map<EntityId, std::vector<ComponentKey>>::const_iterator ecIter =
    this->dataPtr->entityComponents.find(_id);

  if (ecIter == this->dataPtr->entityComponents.end())
    return -1;

  std::vector<ComponentKey>::const_iterator iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
        [&] (const ComponentKey &_key) {return _key.first == _type;});

  if (iter != ecIter->second.end())
    return iter->second;

  return -1;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const EntityId _id, const ComponentTypeId _type) const
{
  std::map<EntityId, std::vector<ComponentKey>>::const_iterator ecIter =
    this->dataPtr->entityComponents.find(_id);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  std::vector<ComponentKey>::const_iterator iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
        [&] (const ComponentKey &_key) {return _key.first == _type;});

  if (iter != ecIter->second.end())
    return this->dataPtr->components.at(iter->first)->Component(iter->second);

  return nullptr;
}

/////////////////////////////////////////////////
void *EntityComponentManager::ComponentImplementation(
    const EntityId _id, const ComponentTypeId _type)
{
  std::map<EntityId, std::vector<ComponentKey>>::const_iterator ecIter =
    this->dataPtr->entityComponents.find(_id);

  if (ecIter == this->dataPtr->entityComponents.end())
    return nullptr;

  std::vector<ComponentKey>::const_iterator iter =
    std::find_if(ecIter->second.begin(), ecIter->second.end(),
        [&] (const ComponentKey &_key) {return _key.first == _type;});

  if (iter != ecIter->second.end())
    return this->dataPtr->components.at(iter->first)->Component(iter->second);

  return nullptr;
}

/////////////////////////////////////////////////
const void *EntityComponentManager::ComponentImplementation(
    const ComponentKey &_key) const
{
  if (this->dataPtr->components.find(_key.first) !=
      this->dataPtr->components.end())
  {
    return this->dataPtr->components.at(_key.first)->Component(_key.second);
  }
  return nullptr;
}

/////////////////////////////////////////////////
void *EntityComponentManager::ComponentImplementation(const ComponentKey &_key)
{
  if (this->dataPtr->components.find(_key.first) !=
      this->dataPtr->components.end())
  {
    return this->dataPtr->components.at(_key.first)->Component(_key.second);
  }
  return nullptr;
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasComponentType(
    const ComponentTypeId _typeId) const
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  return this->HasComponentTypeImpl(_typeId);
}

/////////////////////////////////////////////////
bool EntityComponentManager::HasComponentTypeImpl(
    const ComponentTypeId _typeId) const
{
  return this->dataPtr->components.find(_typeId) !=
    this->dataPtr->components.end();
}

/////////////////////////////////////////////////
void EntityComponentManager::RegisterComponentType(
    const ComponentTypeId _typeId,
    ComponentStorageBase *_type)
{
  igndbg << "Register new component type " << _typeId << ".\n";
  this->dataPtr->components[_typeId].reset(_type);
}

/////////////////////////////////////////////////
void *EntityComponentManager::First(const ComponentTypeId _componentTypeId)
{
  std::map<ComponentTypeId,
    std::unique_ptr<ComponentStorageBase>>::iterator iter =
      this->dataPtr->components.find(_componentTypeId);
  if (iter != this->dataPtr->components.end())
  {
    return iter->second->First();
  }
  return nullptr;
}

//////////////////////////////////////////////////
std::vector<Entity> &EntityComponentManager::Entities() const
{
  return this->dataPtr->entities;
}

//////////////////////////////////////////////////
bool EntityComponentManager::FindView(const std::set<ComponentTypeId> &_types,
    std::map<ComponentTypeKey, View>::iterator &_iter) const
{
  _iter = this->dataPtr->views.find(_types);
  return _iter != this->dataPtr->views.end();
}

//////////////////////////////////////////////////
std::map<ComponentTypeKey, View>::iterator EntityComponentManager::AddView(
    const std::set<ComponentTypeId> &_types, View &&_view) const
{
  // If the view already exists, then the map will return the iterator to
  // the location that prevented the insertion.
  return this->dataPtr->views.insert(
      std::make_pair(_types, std::move(_view))).first;
}

//////////////////////////////////////////////////
void EntityComponentManager::UpdateViews(const EntityId _id)
{
  for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
  {
    // Add/update the entity if it matches the view.
    if (this->EntityMatchesImpl(_id, view.first))
    {
      view.second.AddEntity(_id);
      for (const ComponentTypeId &compTypeId : view.first)
      {
        view.second.AddComponent(_id, compTypeId,
            this->EntityComponentIdFromTypeImpl(_id, compTypeId));
      }
    }
    else if (view.second.entities.find(_id) != view.second.entities.end())
    {
      // Otherwise, remove the entity from the view
      view.second.entities.erase(_id);
      // Remove the entity from the components map
      for (const ComponentTypeId &compTypeId : view.first)
      {
        view.second.components.erase(std::make_pair(_id, compTypeId));
      }
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::RebuildViews()
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  this->RebuildViewsImpl();
}

//////////////////////////////////////////////////
void EntityComponentManager::RebuildViewsImpl()
{
  for (std::pair<const ComponentTypeKey, View> &view : this->dataPtr->views)
  {
    view.second.entities.clear();
    view.second.components.clear();
    // Add all the entities that match the component types to the
    // view.
    for (const Entity &entity : this->dataPtr->entities)
    {
      if (this->EntityMatchesImpl(entity.Id(), view.first))
      {
        view.second.AddEntity(entity.Id());
        // Store pointers to all the components. This recursively adds
        // all the ComponentTypeTs that belong to the entity to the view.
        for (const ComponentTypeId &compTypeId : view.first)
        {
          view.second.AddComponent(entity.Id(), compTypeId,
              this->EntityComponentIdFromTypeImpl(
                entity.Id(), compTypeId));
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void EntityComponentManager::RequestUpdateComponentImpl(
    const EntityId _id, const ComponentTypeId _typeId, const size_t _prioirty,
    const std::any &_value)
{
  this->dataPtr->componentUpdateRequests[std::make_pair(_id, _typeId)] =
      std::make_pair(_prioirty, _value);
}

//////////////////////////////////////////////////
void EntityComponentManager::ProcessUpdateComponentRequests()
{
  std::lock_guard<std::mutex> lock(this->entityMutex);
  for (const auto &[key, val] : this->dataPtr->componentUpdateRequests)
  {
    auto compId = this->EntityComponentIdFromTypeImpl(key.first, key.second);
    if (compId != -1)
    {
      this->dataPtr->components.at(key.second) ->Update(compId, val.second);
    }
  }

  // Clear the update requets
  this->dataPtr->componentUpdateRequests.clear();
}
