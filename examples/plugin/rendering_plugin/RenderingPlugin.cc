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
#include "RenderingPlugin.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>

IGNITION_ADD_PLUGIN(
    sample_system::RenderingPlugin,
    ignition::gazebo::System,
    sample_system::RenderingPlugin::ISystemPostUpdate)
using namespace sample_system;

RenderingPlugin::RenderingPlugin()
{
}

RenderingPlugin::~RenderingPlugin()
{
}

void RenderingPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  auto engine = ignition::rendering::engine("ogre2");
}
