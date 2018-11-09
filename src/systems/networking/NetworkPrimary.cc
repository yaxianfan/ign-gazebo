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
#include "ClientManager.hh"

#include "ignition/common/Console.hh"
#include "ignition/plugin/Register.hh"
#include "ignition/gazebo/Events.hh"

using namespace ignition::gazebo::systems;
using namespace ignition::gazebo::events;

struct NetworkComponent {};

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
void NetworkPrimary::Configure(EntityComponentManager& _ecm,
                               EventManager* _eventMgr)
{
  auto network_entity = _ecm.CreateEntity();
  NetworkComponent network_component;
  _ecm.CreateComponent(network_entity, network_component);

  eventMgr = _eventMgr;
}

/////////////////////////////////////////////////
void NetworkPrimary::Run()
{
  client_thread = std::make_unique<std::thread>(&NetworkPrimary::WaitForClients,
                                                this);
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
bool NetworkPrimary::Running() const
{
  return running;
}

/////////////////////////////////////////////////
void NetworkPrimary::WaitForClients()
{
  while (running && !this->manager->Ready())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    igndbg << "Waiting for all network secondaries to join" << std::endl;
  }
  igndbg << "All network secondaries to joined" << std::endl;

  // Once all clients are discovered, initiate execution.
  worker_thread = std::make_unique<std::thread>(&NetworkPrimary::WorkLoop,
                                                this);
}

/////////////////////////////////////////////////
void NetworkPrimary::WorkLoop()
{
  eventMgr->Emit<events::Pause>(false);

  while (running)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::NetworkPrimary,
                    ignition::gazebo::System,
                    NetworkPrimary::ISystemRunnable,
                    NetworkPrimary::ISystemConfigure)
