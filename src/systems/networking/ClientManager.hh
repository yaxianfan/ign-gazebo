#ifndef IGNITION_GAZEBO_SYSTEMS_CLIENTMANAGER_HH_
#define IGNITION_GAZEBO_SYSTEMS_CLIENTMANAGER_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

#include <ignition/transport/Node.hh>

#include "msgs/client.pb.h"

#include <list>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    struct IGNITION_GAZEBO_VISIBLE ClientInfo {
      std::string uuid;
    };

    class IGNITION_GAZEBO_VISIBLE ClientManager
    {
      public: ClientManager(const std::shared_ptr<ignition::transport::Node> &_node,
                            size_t _expected_num_clients);

      public: ~ClientManager();

      public: bool Ready();

      private: bool registerClient(const msgs::ConnectionRequest &_req,
                                   msgs::ConnectionResponse &_resp);
      private: bool unregisterClient(const msgs::ConnectionRequest &_req,
                                     msgs::ConnectionResponse &_resp);

      private: std::shared_ptr<ignition::transport::Node> node;
      private: size_t expected_num_clients;

      private: std::mutex client_mutex;
      private: std::condition_variable client_cv;
      private: std::map<std::string, ClientInfo> clients;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_CLIENTMANAGER_HH_

