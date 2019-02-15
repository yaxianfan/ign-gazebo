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

#include <gtest/gtest.h>

#include <ignition/msgs/logical_camera_image.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/LogicalCamera.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test LogicalCameraTest system
class LogicalCameraTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem =
        dynamic_cast<MockSystem *>(systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};

std::mutex mutex;
std::vector<msgs::LogicalCameraImage> logicalCameraMsgs;

/////////////////////////////////////////////////
void logicalCameraCb(const msgs::LogicalCameraImage &_msg)
{
  mutex.lock();
  logicalCameraMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the logical camera readings when it faces a box
TEST_F(LogicalCameraTest, LogicalCameraBox)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/logical_camera_sensor.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to logical camera topic
  transport::Node node;
  node.Subscribe("/logical_camera",
      &logicalCameraCb);

  // Run server and verify that we are receiving messages
  size_t iters100 = 100u;
  server.Run(true, iters100, false);
  mutex.lock();
  EXPECT_GT(logicalCameraMsgs.size(), 0u);
  mutex.unlock();

  // Sensor should see box
  std::string boxName = "box";
  math::Pose3d boxPose(1, 0, 0.5, 0, 0, 0);
  math::Pose3d sensorPose(0.05, 0.05, 0.55, 0, 0, 0);
  mutex.lock();
  ignition::msgs::LogicalCameraImage img = logicalCameraMsgs.back();
  EXPECT_EQ(sensorPose, ignition::msgs::Convert(img.pose()));
  EXPECT_EQ(1, img.model().size());
  EXPECT_EQ(boxName, img.model(0).name());
  ignition::math::Pose3d boxPoseCameraFrame = boxPose - sensorPose;
  EXPECT_EQ(boxPoseCameraFrame, ignition::msgs::Convert(img.model(0).pose()));
  mutex.unlock();
}
