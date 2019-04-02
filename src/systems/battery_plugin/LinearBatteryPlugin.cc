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

#include "LinearBatteryPlugin.hh"

#include <string>
#include <functional>

#include <ignition/plugin/Register.hh>

#include <ignition/common/Util.hh>
#include <ignition/common/Time.hh>
#include <ignition/common/Battery.hh>

#include <sdf/Battery.hh>
#include <sdf/Element.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/Battery.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LinearBatteryPluginPrivate
{
  /// \brief Initialize the plugin
  public: void Init();

  /// \brief Reset the plugin
  public: void Reset();

  /// \brief Pointer to world.
  // public: physics::WorldPtr world;

  /// \brief Pointer to battery contained in link.
  public: common::BatteryPtr battery;

  /// \brief Open-circuit voltage.
  /// E(t) = e0 + e1 * Q(t) / c
  public: double e0;
  public: double e1;

  /// \brief Initial battery charge in Ah.
  public: double q0;

  /// \brief Battery capacity in Ah.
  public: double c;

  /// \brief Battery inner resistance in Ohm.
  public: double r;

  /// \brief Current low-pass filter characteristic time in seconds.
  public: double tau;

  /// \brief Raw battery current in A.
  public: double iraw;

  /// \brief Smoothed battery current in A.
  public: double ismooth;

  /// \brief Instantaneous battery charge in Ah.
  public: double q;

  /// \brief Simulation time handled during a single update.
  public: std::chrono::steady_clock::duration stepSize;
};

/////////////////////////////////////////////////
LinearBatteryPlugin::LinearBatteryPlugin()
    : System(), dataPtr(std::make_unique<LinearBatteryPluginPrivate>())
{
  this->dataPtr->iraw = 0.0;
  this->dataPtr->ismooth = 0.0;

  this->dataPtr->e0 = 0.0;
  this->dataPtr->e1 = 0.0;

  this->dataPtr->q0 = 0.0;
  this->dataPtr->q = this->dataPtr->q0;

  this->dataPtr->c = 0.0;
  this->dataPtr->r = 0.0;
  this->dataPtr->tau = 0.0;
}

/////////////////////////////////////////////////
LinearBatteryPlugin::~LinearBatteryPlugin()
{
  this->dataPtr->Reset();

  // This is needed so that common::Battery stops calling the callback function
  //   of this object, when this object is destroyed. Else seg fault in test,
  //   though no seg fault in actual run.
  this->dataPtr->battery->ResetUpdateFunc();
}

/////////////////////////////////////////////////
void LinearBatteryPlugin::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  // Store the pointer to the model
  Model model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Linear battery plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Entity worldEntity = _ecm.EntityByComponents<components::World>(
  //   components::World());
  // auto worldComp = _ecm.ChildrenByComponents(worldEntity,
  //   components::World());


  // Pointer to link that this plugin targets
  Entity linkEntity = kNullEntity;
  if (_sdf->HasElement("link_name"))
  {
    auto linkName = _sdf->Get<std::string>("link_name");

    linkEntity = model.LinkByName(_ecm, linkName);
    IGN_ASSERT(linkEntity != kNullEntity, "Link was NULL");
  }
  else
  {
    ignerr << "link_name not supplied, ignoring LinearBatteryPlugin.\n";
    return;
  }

  if (_sdf->HasElement("open_circuit_voltage_constant_coef"))
    this->dataPtr->e0 = _sdf->Get<double>("open_circuit_voltage_constant_coef");

  if (_sdf->HasElement("open_circuit_voltage_linear_coef"))
    this->dataPtr->e1 = _sdf->Get<double>("open_circuit_voltage_linear_coef");

  if (_sdf->HasElement("initial_charge"))
    this->dataPtr->q0 = _sdf->Get<double>("initial_charge");

  if (_sdf->HasElement("capacity"))
    this->dataPtr->c = _sdf->Get<double>("capacity");

  if (_sdf->HasElement("resistance"))
    this->dataPtr->r = _sdf->Get<double>("resistance");

  if (_sdf->HasElement("smooth_current_tau"))
    this->dataPtr->tau = _sdf->Get<double>("smooth_current_tau");

  if (_sdf->HasElement("battery_name"))
  {
    auto batteryName = _sdf->Get<std::string>("battery_name");

    _ecm.Each<components::Battery, components::Name>(
        [&](const Entity &_batEntity, const components::Battery *_batComp,
            const components::Name *_nameComp) -> bool
        {
          // If parent link matches the target link, and battery name matches
          //   the target battery
          if ((_ecm.ParentEntity(_batEntity) == linkEntity) &&
            (_nameComp->Data() == batteryName))
          {
            this->dataPtr->battery = _batComp->Data();
            return true;
          }
          return true;
        });

    IGN_ASSERT(this->dataPtr->battery, "Battery was NULL");

    if (!this->dataPtr->battery)
    {
      ignerr << "Battery with name[" << batteryName << "] not found. "
            << "The LinearBatteryPlugin will not update its voltage\n";
    }
    else
    {
      this->dataPtr->battery->SetUpdateFunc(
        std::bind(&LinearBatteryPlugin::OnUpdateVoltage, this,
          std::placeholders::_1));
    }
  }
  else
  {
    ignerr << "No <battery_name> specified.\n";
  }

  igndbg << "Battery name: " << this->dataPtr->battery->Name()
         << std::endl;
  igndbg << "Battery voltage: " << this->dataPtr->battery->Voltage()
         << std::endl;

  ignmsg << "LinearBatteryPlugin configured\n";
}

/////////////////////////////////////////////////
void LinearBatteryPluginPrivate::Init()
{
  this->q = this->q0;
}

/////////////////////////////////////////////////
void LinearBatteryPluginPrivate::Reset()
{
  this->iraw = 0.0;
  this->ismooth = 0.0;
  this->Init();
}

//////////////////////////////////////////////////
void LinearBatteryPlugin::Update(const UpdateInfo &_info,
                                 EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->stepSize = _info.dt;
  this->dataPtr->battery->Update();
}

/////////////////////////////////////////////////
double LinearBatteryPlugin::OnUpdateVoltage(
  const common::Battery *_battery)
{
  IGN_ASSERT(_battery != nullptr, "common::Battery is null.");

  double dt = (std::chrono::duration_cast<std::chrono::nanoseconds>(
    this->dataPtr->stepSize).count()) * 1e-9;
  double totalpower = 0.0;
  double k = dt / this->dataPtr->tau;

  if (fabs(_battery->Voltage()) < 1e-3)
    return 0.0;

  for (auto powerLoad : _battery->PowerLoads())
    totalpower += powerLoad.second;

  this->dataPtr->iraw = totalpower / _battery->Voltage();

  this->dataPtr->ismooth = this->dataPtr->ismooth + k *
    (this->dataPtr->iraw - this->dataPtr->ismooth);

  this->dataPtr->q = this->dataPtr->q - ((dt * this->dataPtr->ismooth) /
    3600.0);

  igndbg << "PowerLoads().size(): " << _battery->PowerLoads().size()
         << std::endl;
  igndbg << "voltage: " << (this->dataPtr->e0 + this->dataPtr->e1 * (
    1 - this->dataPtr->q / this->dataPtr->c)
      - this->dataPtr->r * this->dataPtr->ismooth) << std::endl;

  return this->dataPtr->e0 + this->dataPtr->e1 * (
    1 - this->dataPtr->q / this->dataPtr->c)
      - this->dataPtr->r * this->dataPtr->ismooth;
}

IGNITION_ADD_PLUGIN(LinearBatteryPlugin,
                    ignition::gazebo::System,
                    LinearBatteryPlugin::ISystemConfigure,
                    LinearBatteryPlugin::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LinearBatteryPlugin,
  "ignition::gazebo::systems::LinearBatteryPlugin")
