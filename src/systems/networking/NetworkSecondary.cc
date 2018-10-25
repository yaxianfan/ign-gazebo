#include "NetworkSecondary.hh"

#include <ignition/common/Uuid.hh>
#include <ignition/transport/Node.hh>

#include "ignition/plugin/RegisterMore.hh"

#include "msgs/client.pb.h"

using namespace ignition::gazebo::systems;

struct IGNITION_GAZEBO_VERSION_NAMESPACE::NetworkSecondaryPrivate {
  std::shared_ptr<ignition::transport::Node> node;
  bool is_registered {false};
  common::Uuid uuid;
};

NetworkSecondary::NetworkSecondary():
  dataPtr(std::make_unique<NetworkSecondaryPrivate>())
{
  dataPtr->node = std::make_shared<ignition::transport::Node>();
}

NetworkSecondary::~NetworkSecondary()
{
}

void NetworkSecondary::Init(const sdf::ElementPtr &_sdf)
{
  (void)_sdf;

  RegisterWithPrimary();
}

bool NetworkSecondary::RegisterWithPrimary() {
  msgs::ConnectionRequest req;

  req.set_secondary_uuid(this->dataPtr->uuid.String());

  msgs::ConnectionResponse resp;
  bool result;
  unsigned int timeout = 1000;
  bool executed = dataPtr->node->Request("/register", req, timeout, resp, result);

  if (executed) {
    if (result) {
      igndbg << "Registration success" << std::endl;
      dataPtr->is_registered = true;
      return true;
    } else {
      igndbg << "Registration failure" << std::endl;
      return false;
    }
  } else {
    igndbg << "Registration timeout" << std::endl;
    return false;
  }
}

void NetworkSecondary::Run()
{
}

void NetworkSecondary::Stop()
{
}

bool NetworkSecondary::Running()
{
  return false;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::NetworkSecondary,
                    ignition::gazebo::System,
                    NetworkSecondary::ISystemRunnable)
