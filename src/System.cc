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
#include "ignition/gazebo/System.hh"

using namespace ignition::gazebo;

class ignition::gazebo::SystemPrivate
{
  public: explicit SystemPrivate(const std::string &_name,
                                 const SystemTypeId &_system_type)
          : name(_name),
            system_type(_system_type)
  {
  }

  /// \brief Name of the system.
  public: std::string name{""};

  /// \brief Type/Priority of the system.
  public: SystemTypeId system_type{SystemTypeId::UNKNOWN};
};

//////////////////////////////////////////////////
System::System(const std::string &_name, const SystemTypeId &_system_type)
  : dataPtr(new SystemPrivate(_name, _system_type))
{
}

//////////////////////////////////////////////////
System::~System()
{
}

//////////////////////////////////////////////////
void System::Init(EntityQueryRegistrar &/*_registrar*/)
{
}

//////////////////////////////////////////////////
const std::string &System::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void System::SetName(const std::string &_name) const
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
const SystemTypeId &System::SystemType() const
{
  return this->dataPtr->system_type;
}

//////////////////////////////////////////////////
void System::SetSystemType(const SystemTypeId &_system_type)
{
  this->dataPtr->system_type = _system_type;
}

