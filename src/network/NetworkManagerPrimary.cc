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

#include "msgs/peer_control.pb.h"
#include "msgs/simulation_step.pb.h"

#include <algorithm>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"

#include "../components/PerformerLevels.hh"
#include "NetworkManagerPrimary.hh"
#include "NetworkManagerPrivate.hh"
#include "PeerTracker.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerPrimary::NetworkManagerPrimary(
    std::function<void(UpdateInfo &_info)> _stepFunction,
    EntityComponentManager &_ecm, EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  NetworkManager(_stepFunction, _ecm, _eventMgr, _config, _options),
  node(_options)
{
  this->simStepPub = this->node.Advertise<private_msgs::SimulationStep>("step");

  this->node.Subscribe("step_ack", &NetworkManagerPrimary::OnStepAck, this);

  auto eventMgr = this->dataPtr->eventMgr;
  if (eventMgr)
  {
    this->dataPtr->peerRemovedConn = eventMgr->Connect<PeerRemoved>(
        [this](PeerInfo _info)
        {
          if (_info.role == NetworkRole::SimulationSecondary)
          {
            ignmsg << "Secondary removed, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
        });

    this->dataPtr->peerStaleConn = eventMgr->Connect<PeerStale>(
        [this](PeerInfo _info)
        {
          if (_info.role == NetworkRole::SimulationSecondary)
          {
            ignerr << "Secondary went stale, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
        });
  }
  else
  {
    ignwarn << "NetworkManager started without EventManager. "
      << "Distributed environment may not terminate correctly" << std::endl;
  }
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::Handshake()
{
  auto peers = this->dataPtr->tracker->SecondaryPeers();
  for (const auto &peer : peers)
  {
    private_msgs::PeerControl req, resp;
    req.set_enable_sim(true);

    auto sc = std::make_unique<SecondaryControl>();
    sc->id = peer;
    sc->prefix = peer.substr(0, 8);

    bool result;
    std::string topic {sc->prefix + "/control"};
    unsigned int timeout = 5000;

    igndbg << "Registering secondary [" << topic << "]" << std::endl;
    bool executed = this->node.Request(topic, req, timeout, resp, result);

    if (executed)
    {
      if (result)
      {
        ignmsg << "Peer initialized [" << sc->prefix << "]" << std::endl;
        sc->ready = true;
      }
      else
      {
        ignerr << "Peer service call failed [" << sc->prefix << "]"
          << std::endl;
      }
    }
    else
    {
      ignerr << "Peer service call timed out [" << sc->prefix << "], waited "
             << timeout << " ms" << std::endl;
    }

    this->secondaries[sc->prefix] = std::move(sc);
  }
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Ready() const
{
  // The detected number of peers in the "Secondary" role must match
  // the number exepected (set via configuration of environment).
  auto nSecondary = this->dataPtr->tracker->NumSecondary();
  return (nSecondary == this->dataPtr->config.numSecondariesExpected);
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Step(UpdateInfo &_info)
{
  IGN_PROFILE("NetworkManagerPrimary::Step");
  // Check all secondaries have been registered
  bool ready = true;
  for (const auto &secondary : this->secondaries)
  {
    ready &= secondary.second->ready;
  }

  if (!ready)
  {
    ignerr << "Trying to step network primary before all peers are ready."
           << std::endl;
    return false;
  }

  private_msgs::SimulationStep step;

  // Step time
  step.set_iteration(_info.iterations);
  step.set_paused(_info.paused);
  step.mutable_simtime()->CopyFrom(convert<msgs::Time>(_info.simTime));

  auto stepSizeSecNsec = math::durationToSecNsec(_info.dt);
  step.set_stepsize(stepSizeSecNsec.second);

  // Affinities that changed this step
  this->PopulateAffinities(step);

  auto useService{false};

  // Check all secondaries are ready to receive steps - only do this once at
  // startup
  if (useService && !this->SecondariesCanStep())
  {
    return false;
  }

  // Send step to all secondaries in parallel
  this->secondaryStates.clear();

  if (useService)
  {
    for (const auto &secondary : this->secondaries)
    {
      this->node.Request(secondary.second->prefix + "/step", step,
          &NetworkManagerPrimary::OnStepResponse, this);
    }
  }
  else
  {
    this->simStepPub.Publish(step);
  }

  // Block until all secondaries are done
  {
    IGN_PROFILE("Waiting for secondaries");
    while (this->secondaryStates.size() < this->secondaries.size())
    {
      std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    }
  }

  // Update primary state with states received from secondaries
  {
    IGN_PROFILE("Updating primary state");
    for (const auto &msg : this->secondaryStates)
    {
      this->dataPtr->ecm->SetState(msg);
    }
    this->secondaryStates.clear();
  }

  // Step all systems
  this->dataPtr->stepFunction(_info);

  return true;
}

//////////////////////////////////////////////////
std::string NetworkManagerPrimary::Namespace() const
{
  return "";
}

//////////////////////////////////////////////////
std::map<std::string, SecondaryControl::Ptr>
    &NetworkManagerPrimary::Secondaries()
{
  return this->secondaries;
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::OnStepAck(
    const msgs::SerializedState &_msg)
{
  this->secondaryStates.push_back(_msg);
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::OnStepResponse(
    const msgs::SerializedState &_res, const bool _result)
{
  if (_result)
    this->secondaryStates.push_back(_res);
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::SecondariesCanStep() const
{
  static bool allAvailable{false};

  // Only check until it's true
  if (allAvailable)
    return true;

  for (const auto &secondary : this->secondaries)
  {
    std::string service{secondary.second->prefix + "/step"};

    std::vector<transport::ServicePublisher> publishers;
    for (size_t i = 0; i < 50; ++i)
    {
      this->node.ServiceInfo(service, publishers);
      if (!publishers.empty())
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (publishers.empty())
    {
      ignwarn << "Can't step, service [" << service << "] not available."
              << std::endl;
      return false;
    }
  }

  allAvailable = true;
  return true;
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::PopulateAffinities(
    private_msgs::SimulationStep &_msg)
{
  // p: performer
  // l: level
  // s: secondary


  // Group performers per level
  std::map<Entity, std::set<Entity>> lToPNew;

  // Current performer to secondary mapping
  std::map<Entity, std::string> pToSPrevious;

  // Current level to secondary result
  std::map<Entity, std::set<std::string>> lToSNew;

  std::set<Entity> performers;

  // Go through performers and assign affinities
  this->dataPtr->ecm->Each<
        components::Performer,
        components::PerformerLevels>(
    [&](const Entity &_entity,
        const components::Performer *,
        const components::PerformerLevels *_perfLevels) -> bool
    {
      performers.insert(_entity);

      // Get current affinity
      auto currentAffinityComp =
          this->dataPtr->ecm->Component<components::PerformerAffinity>(_entity);
      if (currentAffinityComp)
      {
        pToSPrevious[_entity] = currentAffinityComp->Data();
      }

      for (const auto &level : _perfLevels->Data())
      {
        lToPNew[level].insert(_entity);
        if (currentAffinityComp)
        {
          lToSNew[level].insert(currentAffinityComp->Data());
        }
      }

      return true;
    });

  // First assignment: distribute levels evenly across secondaries
  if (pToSPrevious.empty())
  {
    std::set<Entity> assignedPerformers;
    auto secondaryIt = this->secondaries.begin();
    for (auto [level, performers] : lToPNew)
    {
      for (const auto &performer : performers)
      {
        this->SetAffinity(performer, secondaryIt->second->prefix,
            _msg.add_affinity());
        assignedPerformers.insert(performer);
      }

      // Round-robin levels
      secondaryIt++;
      if (secondaryIt == this->secondaries.end())
      {
        secondaryIt = this->secondaries.begin();
      }
    }

    // Also assign level-less performers
    for (auto performer : performers)
    {
      if (std::find(assignedPerformers.begin(), assignedPerformers.end(),
          performer) != assignedPerformers.end())
      {
        continue;
      }

      this->SetAffinity(performer, secondaryIt->second->prefix, _msg.add_affinity());

      // Round-robin levels
      secondaryIt++;
      if (secondaryIt == this->secondaries.end())
      {
        secondaryIt = this->secondaries.begin();
      }
    }
    return;
  }

  if (pToSPrevious.size() != performers.size())
  {
    ignerr << "There are [" << performers.size()
           << "] performers in total, but [" << pToSPrevious.size()
           << "] performers have been assigned secondaries." << std::endl;
    return;
  }

  // Check for level changes
  for (auto [level, secondaries] : lToSNew)
  {
    // Level is only in one secondary, all good
    if (secondaries.size() <= 1)
      continue;

    ignmsg << "Level [" << level << "] is in [" << secondaries.size()
           << "] secondaries:";
    for (auto s : secondaries)
      std::cout << " [" << s << "] ";
    std::cout << std::endl;

    // Count how many performers in this level are already in each secondary
    std::map<std::string, int> secondaryCounts;
    for (auto performer : lToPNew[level])
    {
      secondaryCounts[pToSPrevious[performer]]++;
    }

    // Choose to keep the level in the secondary with the most performers.
    // If the numbers are the same, any can be chosen.
    std::string chosenSecondary;
    int maxCount{0};
    for (auto [secondary, count] : secondaryCounts)
    {
      if (count > maxCount)
      {
        chosenSecondary = secondary;
        maxCount = count;
      }
    }

    // For each performer in this level, move them to the chosen secondary if
    // not there yet.
    for (auto performer : lToPNew[level])
    {
      auto prevSecondary = pToSPrevious[performer];
      if (prevSecondary == chosenSecondary)
        continue;

      ignmsg << "Reassigning performer [" << performer << "] from secondary ["
             << prevSecondary << "] to secondary [" << chosenSecondary << "]"
             << std::endl;

      // Add new affinity
      this->SetAffinity(performer, chosenSecondary, _msg.add_affinity());
    }
  }
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::SetAffinity(Entity _performer,
    const std::string &_secondary, private_msgs::PerformerAffinity *_msg)
{
  // Get performer model entity and all its children
  auto parentModel = this->dataPtr->ecm->Component<components::ParentEntity>(
      _performer)->Data();
  auto entities = this->Descendants(parentModel);

  // Populate message
  _msg->mutable_entity()->set_id(_performer);
  _msg->mutable_state()->CopyFrom(this->dataPtr->ecm->State(entities));
  _msg->set_secondary_prefix(_secondary);

  // Set component
  this->dataPtr->ecm->RemoveComponent<components::PerformerAffinity>(_performer);

  auto newAffinity = components::PerformerAffinity(_secondary);
  this->dataPtr->ecm->CreateComponent(_performer, newAffinity);
}

//////////////////////////////////////////////////
std::unordered_set<Entity> NetworkManagerPrimary::Descendants(Entity _entity)
{
  std::unordered_set<Entity> descendants;
  descendants.insert(_entity);

  auto adjacents = this->dataPtr->ecm->Entities().AdjacentsFrom(_entity);
  auto current = adjacents.begin();
  while (current != adjacents.end())
  {
    auto id = current->first;

    // Store entity
    descendants.insert(id);

    // Add adjacents to set
    for (auto adj : this->dataPtr->ecm->Entities().AdjacentsFrom(id))
    {
      adjacents.insert(adj);
    }

    // Remove from set
    current = adjacents.erase(current);
  }

  return descendants;
}
