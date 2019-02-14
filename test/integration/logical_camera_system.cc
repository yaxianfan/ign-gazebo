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

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

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
std::vector<msgs::LaserScan> logicalCameraMsgs;

/////////////////////////////////////////////////
void logicalCameraCb(const msgs::LaserScan &_msg)
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

  const std::string sensorName = "logical_camera_sensor";

  // subscribe to lidar topic
  transport::Node node;
  node.Subscribe("/logical_camera", &logicalCameraCb);

  // Run server and verify that we are receiving a message
  // from the lidar
  size_t iters100 = 100u;
  server.Run(true, iters100, false);
  mutex.lock();
  EXPECT_GT(logicalCameraMsgs.size(), 0u);
  mutex.unlock();

  ignition::common::Time waitTime = ignition::common::Time(0.01);
  int i = 0;
  while (i < 300)
  {
    ignition::common::Time::Sleep(waitTime);
    i++;
  }
  mutex.lock();
  EXPECT_GT(logicalCameraMsgs.size(), 0u);
  mutex.unlock();

  // Sensor 1 should see TestBox1
//  mutex.lock();
//  EXPECT_DOUBLE_EQ(logicalCameraMsgs.back().ranges(0), ignition::math::INF_D);
//  EXPECT_NEAR(logicalCameraMsgs.back().ranges(mid), expectedRangeAtMidPointBox1, LASER_TOL);
//  EXPECT_DOUBLE_EQ(logicalCameraMsgs.back().ranges(last), ignition::math::INF_D);
//  EXPECT_EQ(logicalCameraMsgs.back().frame(), "gpu_lidar::gpu_lidar_link");
//  mutex.unlock();
}
