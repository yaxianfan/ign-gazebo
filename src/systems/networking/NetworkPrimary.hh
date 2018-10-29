#ifndef IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_
#define IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/transport/Node.hh>

#include "ClientManager.hh"

#include <memory>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    /// \brief System Plugin for primary distributed simulation instance.
    class IGNITION_GAZEBO_VISIBLE NetworkPrimary:
      public System,
      public ISystemRunnable
    {
      /// \brief Constructor
      public: explicit NetworkPrimary();

      /// \brief Destructor
      public: virtual ~NetworkPrimary();

      /// \brief Initialize the system plugin from SDF file content.
      public: void Init(const sdf::ElementPtr &_sdf) override;

      /// \brief Run the plugin's background threads
      public: void Run() override;

      /// \brief Stop the plugin's background threads
      public: void Stop() override;

      /// \brief Get whether the plugin's background threads are running.
      /// \return True if background threads are executing.
      public: bool Running() override;

      /// \brief Wait for all secondaries to be present before continuing
      private: void WaitForClients();

      /// \brief Main execution loop of the network primary plugin.
      private: void WorkLoop();

      /// \brief Thread responsible for managing secondary clients.
      private: std::unique_ptr<std::thread> client_thread;

      /// \brief Thread responsible for main plugin execution loop
      private: std::unique_ptr<std::thread> worker_thread;

      /// \brief used to indicate that run has been called and that the
      /// background threads are executing.
      private: std::atomic<bool> running;

      /// \brief Transport Node.
      private: std::shared_ptr<ignition::transport::Node> node;

      /// \brief Client Manager.
      private: std::unique_ptr<ClientManager> manager;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition


#endif  // IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_
