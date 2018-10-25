#ifndef IGNITION_GAZEBO_SYSTEMS_NETWORKSECONDARY_HH_
#define IGNITION_GAZEBO_SYSTEMS_NETWORKSECONDARY_HH_

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
    struct NetworkSecondaryPrivate;

    class IGNITION_GAZEBO_VISIBLE NetworkSecondary:
      public System,
      public ISystemRunnable
    {
      public: explicit NetworkSecondary();
      public: virtual ~NetworkSecondary();

      public: void Init(const sdf::ElementPtr &_sdf) override;

      public: void Run() override;
      public: void Stop() override;
      public: bool Running() override;

      private: bool RegisterWithPrimary();

      private: std::unique_ptr<NetworkSecondaryPrivate> dataPtr;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_NETWORKSECONDARY_HH_
