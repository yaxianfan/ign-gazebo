#ifndef IGNITION_GAZEBO_SYSTEMS_NETWORKSECONDARY_HH_
#define IGNITION_GAZEBO_SYSTEMS_NETWORKSECONDARY_HH_

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/common/Uuid.hh>
#include <ignition/transport/Node.hh>

#include <memory>

namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    /// \brief System Plugin for secondary distributed simulation instance.
    class IGNITION_GAZEBO_VISIBLE NetworkSecondary:
      public System,
      public ISystemRunnable
    {
      /// \brief Constructor
      public: explicit NetworkSecondary();

      /// \brief Destructor
      public: virtual ~NetworkSecondary();

      /// \brief Initialize the system plugin from SDF file content.
      public: void Init(const sdf::ElementPtr &_sdf) override;

      /// \brief Run the plugin's background threads
      public: void Run() override;

      /// \brief Stop the plugin's background threads
      public: void Stop() override;

      /// \brief Get whether the plugin's background threads are running.
      /// \return True if background threads are executing.
      public: bool Running() override;

      /// \brief Register this secondary with the primary simulation instance.
      private: bool RegisterWithPrimary();

      /// \brief Transport Node.
      private: std::shared_ptr<ignition::transport::Node> node;

      /// \brief used to indicate that this instance has been registered with
      /// the primary instance.
      private: bool is_registered{false};

      /// \brief Unique identifier of this secondary.
      private: ignition::common::Uuid uuid;
    };
  }  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace systems
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_SYSTEMS_NETWORKSECONDARY_HH_
