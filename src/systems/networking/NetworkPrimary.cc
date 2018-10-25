#include "NetworkPrimary.hh"
#include "ClientManager.hh"

#include <ignition/common/Console.hh>
#include "ignition/plugin/Register.hh"

using namespace ignition::gazebo::systems;

struct IGNITION_GAZEBO_VERSION_NAMESPACE::NetworkPrimaryPrivate {
  std::shared_ptr<ignition::transport::Node> node;
  std::unique_ptr<ClientManager> manager;
};

NetworkPrimary::NetworkPrimary():
  dataPtr(std::make_unique<NetworkPrimaryPrivate>())
{
}

NetworkPrimary::~NetworkPrimary()
{
}

void NetworkPrimary::Init(const sdf::ElementPtr &_sdf)
{
  int num_clients = _sdf->Get<int>("num_clients");
  dataPtr->node = std::make_shared<ignition::transport::Node>();
  dataPtr->manager = std::make_unique<ClientManager>(dataPtr->node, num_clients);
}

void NetworkPrimary::Run()
{
  client_thread = std::thread(&NetworkPrimary::WaitForClients, this);
}

void NetworkPrimary::Stop()
{
  running = false;
  client_thread.join();
  worker_thread.join();
}

bool NetworkPrimary::Running()
{
  return running;
}

void NetworkPrimary::WaitForClients()
{
  while(running && !this->dataPtr->manager->Ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ignmsg << "Waiting for all network secondaries to join" << std::endl;
  }
  ignmsg << "All network secondaries to joined" << std::endl;


  // Once all clients are discovered, initiate execution.
  worker_thread = std::thread(&NetworkPrimary::WorkLoop, this);
}

void NetworkPrimary::WorkLoop()
{
  while(running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::NetworkPrimary,
                    ignition::gazebo::System,
                    NetworkPrimary::ISystemRunnable)
