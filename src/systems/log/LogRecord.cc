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

#include "LogRecord.hh"

#include <sys/stat.h>

#include <string>
#include <fstream>
#include <ctime>

#include <ignition/common/Filesystem.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Recorder.hh>
#include <ignition/transport/Node.hh>

#include <sdf/World.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/Conversions.hh"

using namespace ignition;
using namespace ignition::gazebo::systems;

// Private data class.
class ignition::gazebo::systems::LogRecordPrivate
{
  /// \brief Default directory to record to
  public: static std::string DefaultRecordPath();

  // TODO(mabelmzhang) port to ign-common Filesystem
  /// \brief Generates a path for a file which doesn't collide with existing
  /// files, by appending numbers to it (i.e. (0), (1), ...)
  /// \param[in] _pathAndName Full absolute path and file name up to the
  /// file extension.
  /// \param[in] _extension File extension, such as "ddf".
  /// \return Full path with name and extension, which doesn't collide with
  /// existing files
  public: std::string UniqueFilePath(const std::string &_pathAndName,
    const std::string &_extension);

  /// \brief Unique directory path to not overwrite existing directory
  /// \param[in] _pathAndName Full absolute path
  public: std::string UniqueDirectoryPath(const std::string &_dir);

  /// \brief Clock used to timestamp recorded messages with sim time
  /// coming from /clock topic. This is not the timestamp on the header,
  /// rather a logging-specific stamp.
  /// In case there's disagreement between these stamps, the one in the
  /// header should be used.
  public: std::unique_ptr<transport::NetworkClock> clock;

  /// \brief Ignition transport recorder
  public: transport::log::Recorder recorder;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Log publisher.
  public: transport::Node::Publisher logPub;
};

//////////////////////////////////////////////////
std::string LogRecordPrivate::DefaultRecordPath()
{
  std::string home;
  common::env(IGN_HOMEDIR, home);

  std::time_t timestamp = std::time(nullptr);

  std::string path = common::joinPaths(home,
    ".ignition", "gazebo", "log", std::to_string(timestamp));

  return path;
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::UniqueFilePath(const std::string &_pathAndName,
  const std::string &_extension)
{
  std::string result = _pathAndName + "." + _extension;
  int count = 1;

  // Check if file exists and change name accordingly
  while (common::exists(result.c_str()))
  {
    result = _pathAndName + "(" + std::to_string(count++) + ").";
    result += _extension;
  }

  return result;
}

//////////////////////////////////////////////////
std::string LogRecordPrivate::UniqueDirectoryPath(const std::string &_dir)
{
  std::string result = _dir;
  int count = 1;

  // Check if file exists and change name accordingly
  while (common::exists(result.c_str()))
  {
    result = _dir + "(" + std::to_string(count++) + ")";
  }

  return result;
}

//////////////////////////////////////////////////
LogRecord::LogRecord()
  : System(), dataPtr(std::make_unique<LogRecordPrivate>())
{
}

//////////////////////////////////////////////////
LogRecord::~LogRecord()
{
  // Use ign-transport directly
  this->dataPtr->recorder.Stop();

  ignmsg << "Stopping recording" << std::endl;
}

//////////////////////////////////////////////////
void LogRecord::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  // Get directory paths from SDF params
  auto logPath = _sdf->Get<std::string>("path");

  // If unspecified, or specified is not a directory, use default directory
  if (logPath.empty() ||
      (common::exists(logPath) && !common::isDirectory(logPath)))
  {
    logPath = this->dataPtr->DefaultRecordPath();
    ignmsg << "Unspecified or invalid log path to record to. "
      << "Recording to default location [" << logPath << "]" << std::endl;
  }

  // If directoriy already exists, do not overwrite
  if (common::exists(logPath))
  {
    logPath = this->dataPtr->UniqueDirectoryPath(logPath);
    ignwarn << "Log path already exists on disk! "
      << "Recording instead to [" << logPath << "]" << std::endl;
  }

  // Create log directory
  if (!common::exists(logPath))
  {
    common::createDirectories(logPath);
  }

  // Append file names
  std::string dbPath = common::joinPaths(logPath, "state.tlog");

  // Temporary for recording sdf string
  std::string sdfPath = common::joinPaths(logPath, "state.sdf");

  // Record SDF as a string.

  // TODO(mabelmzhang): For now, just dumping a big string to a text file,
  // until we have a message for the SDF.
  if (nullptr != _sdf && nullptr != _sdf->GetParent())
  {
    std::ofstream ofs(sdfPath);

    // Go up to root of SDF, to output entire SDF file
    sdf::ElementPtr sdfRoot = _sdf->GetParent();
    while (sdfRoot->GetParent() != nullptr)
    {
      sdfRoot = sdfRoot->GetParent();
    }
    ofs << sdfRoot->ToString("");
    ignmsg << "Saved initial SDF file to [" << sdfPath << "]" << std::endl;
  }
  else
  {
    ignerr << "Failed to save initial SDF world file." << std::endl;
  }

  ignmsg << "Recording to log file [" << dbPath << "]" << std::endl;

  // Use ign-transport directly
  auto worldName = _ecm.Component<components::Name>(_entity)->Data();

  auto logTopic = "/world/" + worldName + "/log";
  this->dataPtr->recorder.AddTopic(logTopic);

  // Timestamp messages with sim time from clock topic
  // Note that the message headers should also have a timestamp
  auto clockTopic = "/world/" + worldName + "/clock";
  this->dataPtr->clock = std::make_unique<transport::NetworkClock>(clockTopic,
      transport::NetworkClock::TimeBase::SIM);
  this->dataPtr->recorder.Sync(this->dataPtr->clock.get());

  // This calls Log::Open() and loads sql schema
  this->dataPtr->recorder.Start(dbPath);

  // Publisher to log topic
  this->dataPtr->logPub = this->dataPtr->node.Advertise<msgs::SerializedState>(
      logTopic);
}

//////////////////////////////////////////////////
void LogRecord::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // TODO(anyone) Support getting only user-selected components
  // Get current state and timestamp it
  auto stateMsg = _ecm.State();
  stateMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Publish it to the log topic and let the recorder do the rest
  this->dataPtr->logPub.Publish(stateMsg);
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::LogRecord,
                    ignition::gazebo::System,
                    LogRecord::ISystemConfigure,
                    LogRecord::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::LogRecord,
                          "ignition::gazebo::systems::LogRecord")
