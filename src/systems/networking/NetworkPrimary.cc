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
#include "NetworkPrimary.hh"

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include "ClientManager.hh"

using namespace ignition::gazebo::systems;

/////////////////////////////////////////////////
NetworkPrimary::NetworkPrimary():
  node(std::make_shared<ignition::transport::Node>())
{
}

/////////////////////////////////////////////////
NetworkPrimary::~NetworkPrimary()
{
}

/////////////////////////////////////////////////
void NetworkPrimary::Init(const sdf::ElementPtr &_sdf)
{
  int num_clients = _sdf->Get<int>("num_clients");
  manager = std::make_unique<ClientManager>(node, num_clients);
}

/////////////////////////////////////////////////
void NetworkPrimary::Run()
{
  client_thread = std::make_unique<std::thread>(
      &NetworkPrimary::WaitForClients, this);
}

/////////////////////////////////////////////////
void NetworkPrimary::Stop()
{
  running = false;
  if (client_thread) {
    client_thread->join();
    client_thread.release();
  }

  if (worker_thread) {
    worker_thread->join();
    worker_thread.release();
  }
}

/////////////////////////////////////////////////
bool NetworkPrimary::Running()
{
  return running;
}

/////////////////////////////////////////////////
void NetworkPrimary::WaitForClients()
{
  while (running && !this->manager->Ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ignmsg << "Waiting for all network secondaries to join" << std::endl;
  }

  if (!running) {
    ignmsg << "Interrupted while waiting for network secondaries to join" <<
      std::endl;
    return;
  }

  ignmsg << "All network secondaries joined" << std::endl;

  // Once all clients are discovered, initiate execution.
  worker_thread = std::make_unique<std::thread>(&NetworkPrimary::WorkLoop,
                                                this);
}

/////////////////////////////////////////////////
void NetworkPrimary::WorkLoop()
{
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::NetworkPrimary,
                    ignition::gazebo::System,
                    NetworkPrimary::ISystemRunnable)
