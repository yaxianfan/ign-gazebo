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

#include <gtest/gtest.h>

#include <cstdlib>
#include <ignition/common/Console.hh>

#include "PeerTracker.hh"
#include "ignition/gazebo/EventManager.hh"

using namespace ignition::gazebo;

//////////////////////////////////////////////////
TEST(PeerTracker, PeerTracker)
{
  ignition::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  std::atomic<int> peers = 0;
  auto added = eventMgr.Connect<PeerAdded>([&peers](PeerInfo _peer)
  {
    (void) _peer;
    peers++;
  });

  auto removed = eventMgr.Connect<PeerRemoved>([&peers](PeerInfo _peer)
  {
    (void) _peer;
    peers--;
  });

  auto tracker1 = PeerTracker(&eventMgr);
  auto tracker2 = PeerTracker();
  auto tracker3 = PeerTracker();
  auto tracker4 = PeerTracker();
  auto tracker5 = PeerTracker();
  auto tracker6 = PeerTracker();

  auto info1 = std::make_shared<PeerInfo>();
  info1->role = NetworkRole::SimulationPrimary;

  auto info2 = std::make_shared<PeerInfo>();
  info2->role = NetworkRole::SimulationSecondary;

  auto info3 = std::make_shared<PeerInfo>();
  info3->role = NetworkRole::SimulationSecondary;

  auto info4 = std::make_shared<PeerInfo>();
  info4->role = NetworkRole::ReadOnly;

  auto info5 = std::make_shared<PeerInfo>();
  info5->role = NetworkRole::ReadOnly;

  auto info6 = std::make_shared<PeerInfo>();
  info6->role = NetworkRole::None;

  tracker1.Connect(info1);
  EXPECT_EQ(0, peers);

  tracker2.Connect(info2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(1, peers);

  tracker3.Connect(info3);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(2, peers);

  tracker4.Connect(info4);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(3, peers);

  tracker5.Connect(info5);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(4, peers);

  tracker6.Connect(info6);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(5, peers);

  // Allow all the heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // All counts exclude self.
  EXPECT_EQ(5u, tracker1.NumPeers());
  EXPECT_EQ(5u, tracker2.NumPeers());
  EXPECT_EQ(5u, tracker3.NumPeers());
  EXPECT_EQ(5u, tracker4.NumPeers());
  EXPECT_EQ(5u, tracker5.NumPeers());

  EXPECT_EQ(0u, tracker1.NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(2u, tracker1.NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(2u, tracker1.NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker1.NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker2.NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(1u, tracker2.NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(2u, tracker2.NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker2.NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker3.NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(1u, tracker3.NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(2u, tracker3.NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker3.NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker4.NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(2u, tracker4.NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(1u, tracker4.NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker4.NumPeers(NetworkRole::None));

  EXPECT_EQ(1u, tracker5.NumPeers(NetworkRole::SimulationPrimary));
  EXPECT_EQ(2u, tracker5.NumPeers(NetworkRole::SimulationSecondary));
  EXPECT_EQ(1u, tracker5.NumPeers(NetworkRole::ReadOnly));
  EXPECT_EQ(1u, tracker5.NumPeers(NetworkRole::None));

  tracker6.Disconnect();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(4, peers);

  tracker5.Disconnect();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(3, peers);

  tracker4.Disconnect();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(2, peers);

  tracker3.Disconnect();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(1, peers);

  tracker2.Disconnect();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(0, peers);

  tracker1.Disconnect();
}

//////////////////////////////////////////////////
TEST(PeerTracker, PeerTrackerStale)
{
  ignition::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  // Tracker with artificially short timeout.
  auto tracker1 = PeerTracker(&eventMgr);
  tracker1.SetHeartbeatPeriod(std::chrono::milliseconds(10));
  tracker1.SetStaleMultiplier(1);

  auto tracker2 = PeerTracker(&eventMgr);
  tracker2.SetHeartbeatPeriod(std::chrono::milliseconds(100));

  auto info1 = std::make_shared<PeerInfo>();
  info1->role = NetworkRole::SimulationPrimary;

  auto info2 = std::make_shared<PeerInfo>();
  info2->role = NetworkRole::SimulationSecondary;

  std::atomic<int> stalePeers = 0;
  auto stale = eventMgr.Connect<PeerStale>([&](PeerInfo _peer)
  {
    EXPECT_EQ(_peer.id, info2->id);
    stalePeers++;
  });

  tracker1.Connect(info1);
  tracker2.Connect(info2);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(1, stalePeers);

  // Expect while tracker1 can see tracker2, the opposite is
  // not true.
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());

  // PeerTracker will print a debug message when DISCONNECTING message is
  // received from stale peer
}

//////////////////////////////////////////////////
TEST(PeerTracker, Partitioned)
{
  ignition::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  auto options1 = ignition::transport::NodeOptions();
  options1.SetPartition("p1");
  auto tracker1 = PeerTracker(&eventMgr, options1);

  auto info1 = std::make_shared<PeerInfo>();
  info1->role = NetworkRole::SimulationPrimary;
  tracker1.Connect(info1);

  auto options2 = ignition::transport::NodeOptions();
  options2.SetPartition("p2");
  auto tracker2 = PeerTracker(&eventMgr, options2);

  auto info2 = std::make_shared<PeerInfo>();
  info2->role = NetworkRole::SimulationPrimary;
  tracker2.Connect(info2);

  // Allow all the heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Trackers should not detect peers in different partitions
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(0u, tracker2.NumPeers());

  auto tracker3 = PeerTracker(&eventMgr, options1);

  auto info3 = std::make_shared<PeerInfo>();
  info3->role = NetworkRole::SimulationSecondary;
  tracker3.Connect(info3);

  auto tracker4 = PeerTracker(&eventMgr, options2);

  auto info4 = std::make_shared<PeerInfo>();
  info4->role = NetworkRole::SimulationSecondary;
  tracker4.Connect(info4);

  // Allow some time for heartbeats to propagate
  // TODO(mjcarroll): Send heartbeats on announce
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should detect peers in the same partition.
  EXPECT_EQ(1u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());
  EXPECT_EQ(1u, tracker3.NumPeers());
  EXPECT_EQ(1u, tracker4.NumPeers());
}

//////////////////////////////////////////////////
TEST(PeerTracker, Namespaced)
{
  ignition::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  auto options1 = ignition::transport::NodeOptions();
  options1.SetNameSpace("ns1");
  auto tracker1 = PeerTracker(&eventMgr, options1);

  auto info1 = std::make_shared<PeerInfo>();
  info1->role = NetworkRole::SimulationPrimary;
  tracker1.Connect(info1);

  auto options2 = ignition::transport::NodeOptions();
  options2.SetNameSpace("ns2");
  auto tracker2 = PeerTracker(&eventMgr, options2);

  auto info2 = std::make_shared<PeerInfo>();
  info2->role = NetworkRole::SimulationPrimary;
  tracker2.Connect(info2);

  // Allow some time for heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should not detect peers in different namespaces
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(0u, tracker2.NumPeers());

  auto tracker3 = PeerTracker(&eventMgr, options1);

  auto info3 = std::make_shared<PeerInfo>();
  info3->role = NetworkRole::SimulationSecondary;
  tracker3.Connect(info3);

  auto tracker4 = PeerTracker(&eventMgr, options2);

  auto info4 = std::make_shared<PeerInfo>();
  info4->role = NetworkRole::SimulationSecondary;
  tracker4.Connect(info4);

  // Allow some time for heartbeats to propagate
  // TODO(mjcarroll): Send heartbeats on announce
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should detect peers in the same partition.
  EXPECT_EQ(1u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());
  EXPECT_EQ(1u, tracker3.NumPeers());
  EXPECT_EQ(1u, tracker4.NumPeers());
}

//////////////////////////////////////////////////
// Only on Linux for the moment
#ifdef  __linux__
TEST(PeerTracker, PartitionedEnv)
{
  ignition::common::Console::SetVerbosity(4);
  EventManager eventMgr;

  setenv("IGN_PARTITION", "p1", 1);
  auto tracker1 = PeerTracker(&eventMgr);
  auto info1 = std::make_shared<PeerInfo>();
  info1->role = NetworkRole::SimulationPrimary;
  tracker1.Connect(info1);

  setenv("IGN_PARTITION", "p2", 1);
  auto tracker2 = PeerTracker(&eventMgr);
  auto info2 = std::make_shared<PeerInfo>();
  info2->role = NetworkRole::SimulationPrimary;
  tracker2.Connect(info2);

  // Allow some time for heartbeats to propagate
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should not detect peers in different partitions
  EXPECT_EQ(0u, tracker1.NumPeers());
  EXPECT_EQ(0u, tracker2.NumPeers());

  setenv("IGN_PARTITION", "p1", 1);
  auto tracker3 = PeerTracker(&eventMgr);
  auto info3 = std::make_shared<PeerInfo>();
  info3->role = NetworkRole::SimulationSecondary;
  tracker3.Connect(info3);

  setenv("IGN_PARTITION", "p2", 1);
  auto tracker4 = PeerTracker(&eventMgr);
  auto info4 = std::make_shared<PeerInfo>();
  info4->role = NetworkRole::SimulationSecondary;
  tracker4.Connect(info4);

  // Allow some time for heartbeats to propagate
  // TODO(mjcarroll): Send heartbeats on announce
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Trackers should detect peers in the same partition.
  EXPECT_EQ(1u, tracker1.NumPeers());
  EXPECT_EQ(1u, tracker2.NumPeers());
  EXPECT_EQ(1u, tracker3.NumPeers());
  EXPECT_EQ(1u, tracker4.NumPeers());

  unsetenv("IGN_PARTITION");
}
#endif
