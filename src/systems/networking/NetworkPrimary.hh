#ifndef IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_
#define IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

#include <memory>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    struct NetworkPrimaryPrivate;

    class IGNITION_GAZEBO_VISIBLE NetworkPrimary:
      public System,
      public ISystemRunnable
    {
      public: explicit NetworkPrimary();
      public: virtual ~NetworkPrimary();

      public: void Init(const sdf::ElementPtr &_sdf) override;

      public: void Run() override;
      public: void Stop() override;
      public: bool Running() override;

      private: void WaitForClients();
      private: void WorkLoop();

      private: std::thread client_thread;
      private: std::thread worker_thread;
      private: std::atomic<bool> running;

      private: std::unique_ptr<NetworkPrimaryPrivate> dataPtr;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition


#endif  // IGNITION_GAZEBO_SYSTEMS_NETWORKPRIMARY_HH_
