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
#ifndef IGNITION_GAZEBO_SERVERCONFIG_HH_
#define IGNITION_GAZEBO_SERVERCONFIG_HH_

#include <chrono>
#include <list>
#include <memory>
#include <optional> // NOLINT(*)
#include <string>
#include <sdf/Element.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class ServerConfigPrivate;

    /// \class ServerConfig ServerConfig.hh ignition/gazebo/ServerConfig.hh
    /// \brief Configuration parameters for a Server. An instance of this
    /// object can be used to construct a Server with a particular
    /// configuration.
    class IGNITION_GAZEBO_VISIBLE ServerConfig
    {
      class PluginInfoPrivate;
      /// \brief Information about a plugin that should be loaded by the
      /// server.
      /// \sa const std::list<PluginInfo> &Plugins() const
      public: class PluginInfo
      {
        /// \brief Default constructor.
        public: PluginInfo();

        /// \brief Destructor.
        public: ~PluginInfo();

        /// \brief Constructor with plugin information specified.
        /// \param[in] _entityName Name of the entity which should receive
        /// this plugin. The name is used in conjuction with _entityType to
        /// uniquely identify an entity.
        /// \param[in] _entityType Entity type which should receive  this
        /// plugin. The type is used in conjuction with _entityName to
        /// uniquely identify an entity.
        /// \param[in] _filename Plugin library filename.
        /// \param[in] _name Name of the interface within the plugin library
        /// to load.
        /// \param[in] _sdf Plugin XML elements associated with this plugin.
        public: PluginInfo(const std::string &_entityName,
                           const std::string &_entityType,
                           const std::string &_filename,
                           const std::string &_name,
                           const sdf::ElementPtr &_sdf);

        /// \brief Copy constructor.
        /// \param[in] _info Plugin to copy.
        public: PluginInfo(const PluginInfo &_info);

        /// \brief Equal operator.
        /// \param[in] _info PluginInfo to copy.
        /// \return Reference to this class.
        public: PluginInfo &operator=(const PluginInfo &_info);

        /// \brief Get the name of the entity which should receive
        /// this plugin. The name is used in conjuction with _entityType to
        /// uniquely identify an entity.
        /// \return Entity name.
        public: const std::string &EntityName() const;

        /// \brief Get the entity type which should receive  this
        /// plugin. The type is used in conjuction with EntityName to
        /// uniquely identify an entity.
        /// \return Entity type string.
        public: const std::string &EntityType() const;

        /// \brief Get the plugin library filename.
        /// \return Plugin library filename.
        public: const std::string &Filename() const;

        /// \brief Name of the interface within the plugin library
        /// to load.
        /// \return Interface name.
        public: const std::string &Name() const;

        /// \brief Plugin XML elements associated with this plugin.
        /// \return SDF pointer.
        public: const sdf::ElementPtr &Sdf() const;

        /// \brief Private data pointer
        private: std::unique_ptr<ServerConfig::PluginInfoPrivate> dataPtr;
      };

      /// \brief Constructor
      public: ServerConfig();

      /// \brief Copy constructor.
      /// \param[in] _config ServerConfig to copy.
      public: ServerConfig(const ServerConfig &_config);

      /// \brief Destructor
      public: ~ServerConfig();

      /// \brief Set an SDF file. The SDF parser will attempt to load the
      /// provided file. If the file fails to load, then the stored SDF file
      /// will remain unchanged and a false value will be returned. You can
      /// override the check using the _force parameter.
      /// \param[in] _file Full path to an SDF file.
      /// \param[in] _force Force the stored SDF file, bypassing the SDF
      /// parser check.
      /// \return True if the provided file was successfully found and
      /// parsed, false otherwise.
      public: bool SetSdfFile(const std::string &_file);

      /// \brief Get the SDF file that has been set. An empty string will be
      /// returned if an SDF file has not been set.
      /// \return The full path to the SDF file, or empty string.
      public: std::string SdfFile() const;

      /// \brief Set the update rate in Hertz. Value <=0 are ignored.
      /// \param[in] _hz The desired update rate of the server in Hertz.
      public: void SetUpdateRate(const double &_hz);

      /// \brief Get the update rate in Hertz.
      /// \return The desired update rate of the server in Hertz, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<double> UpdateRate() const;

      /// \brief Get whether the server is using the level system
      /// \return True if the server is set to use the level system
      public: bool UseLevels() const;

      /// \brief Get whether the server is using the level system.
      /// \param[in] _levels Value to set.
      public: void SetUseLevels(const bool _levels);

      /// \brief Get the update period duration.
      /// \return The desired update period, or nullopt if
      /// an UpdateRate has not been set.
      public: std::optional<std::chrono::steady_clock::duration>
              UpdatePeriod() const;

      /// \brief Path to where simulation resources, such as models downloaded
      /// from fuel.ignitionrobotics.org, should be stored.
      /// \return Path to a location on disk. An empty string indicates that
      /// the default value will be used, which is currently
      /// ~/.ignition/fuel.
      public: const std::string &ResourceCache() const;

      /// \brief Set the path to where simulation resources, such as models
      /// downloaded from fuel.ignitionrobotics.org, should be stored.
      /// \param[in] _path Path to a location on disk. An empty string
      /// indicates that the default value will be used, which is currently
      /// ~/.ignition/fuel.
      public: void SetResourceCache(const std::string &_path);

      /// \brief Instruct simulation to attach a plugin to a specific
      /// entity when simulation starts.
      /// \param[in] _info Information about the plugin to load.
      public: void AddPlugin(const PluginInfo &_info);

      /// \brief Get all the plugins that should be loaded.
      /// \return A list of all the plugins specified via
      /// AddPlugin(const PluginInfo &).
      public: const std::list<PluginInfo> &Plugins() const;

      /// \brief Equal operator.
      /// \param[in] _cfg ServerConfig to copy.
      /// \return Reference to this class.
      public: ServerConfig &operator=(const ServerConfig &_cfg);

      /// \brief Private data pointer
      private: std::unique_ptr<ServerConfigPrivate> dataPtr;
    };
    }
  }
}

#endif
