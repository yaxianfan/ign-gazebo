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
#include <algorithm>
#include <set>

#include "ignition/gazebo/EntityQuery.hh"
#include "ignition/gazebo/ComponentManager.hh"

using namespace ignition;
using namespace gazebo;

// Private data class
class ignition::gazebo::EntityQueryPrivate
{
  /// \brief list of component types that must be present on entities
  public: std::set<ComponentTypeId> componentTypes;
};

/////////////////////////////////////////////////
EntityQuery::EntityQuery()
: dataPtr(new EntityQueryPrivate())
{
}

/////////////////////////////////////////////////
EntityQuery::~EntityQuery()
{
}

/////////////////////////////////////////////////
bool EntityQuery::AddComponentType(const ComponentType &_type)
{
  return this->AddComponentType(_type.TypeId());
}

/////////////////////////////////////////////////
bool EntityQuery::AddComponentType(const ComponentTypeId _type)
{
  if (_type != kComponentTypeIdInvalid)
  {
    this->dataPtr->componentTypes.insert(_type);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
bool EntityQuery::operator==(const EntityQuery &_query) const
{
  return std::equal(this->dataPtr->componentTypes.begin(),
                    this->dataPtr->componentTypes.end(),
                    _query.dataPtr->componentTypes.begin());
}

/////////////////////////////////////////////////
const std::set<ComponentTypeId> &EntityQuery::ComponentTypes() const
{
  return this->dataPtr->componentTypes;
}

/////////////////////////////////////////////////
bool EntityQuery::Empty() const
{
  return this->dataPtr->componentTypes.empty();
}
