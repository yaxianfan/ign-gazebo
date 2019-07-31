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

#include <set>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/common/Time.hh>
#include <ignition/math/Helpers.hh>

#include <ignition/rendering/Scene.hh>
#include <ignition/sensors/CameraSensor.hh>
#include <ignition/sensors/RenderingSensor.hh>
#include <ignition/sensors/Manager.hh>

#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/rendering/RenderUtil.hh"

#include "Sensors.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::SensorsPrivate
{
  /// \brief Sensor manager object. This manages the lifecycle of the
  /// instantiated sensors.
  public: sensors::Manager sensorManager;

  /// \brief used to store whether rendering objects have been created.
  public: bool initialized = false;

  /// \brief Main rendering interface
  public: RenderUtil renderUtil;

  /// \brief Unique set of sensor ids
  // TODO(anyone) Remove element when sensor is deleted
  public: std::set<sensors::SensorId> sensorIds;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;

  /// \brief Keep track of cameras, in case we need to handle stereo cameras.
  /// Key: Camera's parent scoped name
  /// Value: Pointer to camera
  // TODO(anyone) Remove element when sensor is deleted
  public: std::map<std::string, sensors::CameraSensor *> cameras;

};

//////////////////////////////////////////////////
Sensors::Sensors() : System(), dataPtr(std::make_unique<SensorsPrivate>())
{
}

//////////////////////////////////////////////////
Sensors::~Sensors() = default;

//////////////////////////////////////////////////
void Sensors::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  // Setup rendering
  std::string engineName =
      _sdf->Get<std::string>("render_engine", "ogre2").first;

  this->dataPtr->renderUtil.SetEngineName(engineName);
  this->dataPtr->renderUtil.SetEnableSensors(true,
      std::bind(&Sensors::CreateSensor, this,
      std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
void Sensors::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Sensors::PostUpdate");
  // Only initialize if there are rendering sensors
  if (!this->dataPtr->initialized &&
      (_ecm.HasComponentType(components::Camera::typeId) ||
       _ecm.HasComponentType(components::DepthCamera::typeId) ||
       _ecm.HasComponentType(components::GpuLidar::typeId) ||
       _ecm.HasComponentType(components::RgbdCamera::typeId)))
  {
    this->dataPtr->renderUtil.Init();
    this->dataPtr->scene = this->dataPtr->renderUtil.Scene();
    this->dataPtr->initialized = true;
  }

  if (!this->dataPtr->initialized)
    return;

  this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);

  // update scene graph
  this->dataPtr->renderUtil.Update();

  // udate rendering sensors
  auto time = math::durationToSecNsec(_info.simTime);
  auto t = common::Time(time.first, time.second);

  // enable sensors if they need to be updated
  std::vector<sensors::RenderingSensor *> activeSensors;

  for (auto id : this->dataPtr->sensorIds)
  {
    sensors::Sensor *s = this->dataPtr->sensorManager.Sensor(id);
    sensors::RenderingSensor *rs = dynamic_cast<sensors::RenderingSensor *>(s);
    if (rs && rs->NextUpdateTime() <= t)
    {
      activeSensors.push_back(rs);
    }
  }

  if (activeSensors.empty())
    return;

  common::Time tn = common::Time::SystemTime();

  // Update the scene graph manually to improve performance
  // We only need to do this once per frame It is important to call
  // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
  // so we don't waste cycles doing one scene graph update per sensor
  {
    IGN_PROFILE("Sensors::PostUpdate Pre-render scene");
    this->dataPtr->scene->PreRender();
  }

  // publish data
  {
    IGN_PROFILE("Sensors::PostUpdate Run sensor manager");
    this->dataPtr->sensorManager.RunOnce(t);
  }

  common::Time dt = common::Time::SystemTime() - tn;
  // igndbg << "sensor update time: " << dt.Double() << std::endl;
}

//////////////////////////////////////////////////
std::string Sensors::CreateSensor(const sdf::Sensor &_sdf,
    const std::string &_parentName)
{
  if (_sdf.Type() == sdf::SensorType::NONE)
  {
    ignerr << "Unable to create sensor. SDF sensor type is NONE." << std::endl;
    return std::string();
  }

  // Create within ign-sensors
  auto sensorId = this->dataPtr->sensorManager.CreateSensor(_sdf);
  auto sensor = this->dataPtr->sensorManager.Sensor(sensorId);

  if (nullptr == sensor || sensors::NO_SENSOR == sensor->Id())
  {
    ignerr << "Failed to create sensor [" << _sdf.Name()
           << "]" << std::endl;
    return std::string();
  }

  this->dataPtr->sensorIds.insert(sensorId);

  // Set the scene so it can create the rendering sensor
  auto renderingSensor =
      dynamic_cast<sensors::RenderingSensor *>(sensor);
  renderingSensor->SetScene(this->dataPtr->scene);
  renderingSensor->SetParent(_parentName);
  renderingSensor->SetManualSceneUpdate(true);

  // Special case for stereo cameras
  auto cameraSensor = dynamic_cast<sensors::CameraSensor *>(sensor);
  if (nullptr != cameraSensor)
  {
    // Parent
    auto parent = cameraSensor->Parent();

    // If parent has other camera children, set the baseline.
    // For stereo pairs, the baseline for the left camera is zero, and for the
    // right camera it's the distance between them.
    // For more than 2 cameras, the first camera's baseline is zero and the
    // others have the distance between them.
    if (this->dataPtr->cameras.find(parent) !=
        this->dataPtr->cameras.end())
    {
      // TODO(anyone) This is safe because we're not removing sensors
      // First camera added to the parent link
      auto leftCamera = this->dataPtr->cameras[parent];
      auto rightCamera = cameraSensor;

      // If cameras have right / left topic, use that to decide which is which
      if (leftCamera->Topic().find("right") != std::string::npos &&
          rightCamera->Topic().find("left") != std::string::npos)
      {
        std::swap(rightCamera, leftCamera);
      }

      // Camera sensor's Y axis corresponds to the optical X axis
      auto baseline = abs(rightCamera->Pose().Pos().Y() -
                          leftCamera->Pose().Pos().Y());
      rightCamera->SetBaseline(baseline);
    }
    else
    {
      this->dataPtr->cameras[parent] = cameraSensor;
    }
  }

  return sensor->Name();
}

IGNITION_ADD_PLUGIN(Sensors, System,
  Sensors::ISystemConfigure,
  Sensors::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Sensors, "ignition::gazebo::systems::Sensors")
