/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_EVENTMANAGER_HH_
#define IGNITION_GAZEBO_EVENTMANAGER_HH_

#include <functional>
#include <memory>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include <ignition/common/Event.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    // Forward declarations.
    class IGNITION_GAZEBO_HIDDEN EventManagerPrivate;

    class IGNITION_GAZEBO_VISIBLE EventManager
    {
      /// \brief Constructor
      public: EventManager();

      /// \brief Destructor
      public: ~EventManager();

      public: template <typename E>
              ignition::common::ConnectionPtr
              Connect(const typename E::CallbackT &_subscriber)
              {
                if (events.find(typeid(E)) == events.end()) {
                  events[typeid(E)] = std::make_unique<E>();
                }

                E* event_ptr = dynamic_cast<E*>(events[typeid(E)].get());
                if (event_ptr != nullptr)
                {
                  return event_ptr->Connect(_subscriber);
                }
                return nullptr;
              }

      public: template <typename E, typename ... Args>
              void Emit(Args && ... args)
              {
                if (events.find(typeid(E)) == events.end())
                {
                  return;
                }
                E* event_ptr = dynamic_cast<E*>(events[typeid(E)].get());
                if (event_ptr != nullptr)
                {
                  event_ptr->Signal(std::forward<Args>(args) ...);
                }
              }

      private: using TypeInfoRef = std::reference_wrapper<const std::type_info>;

      private: struct Hasher {
                 std::size_t operator()(TypeInfoRef code) const
                 {
                   return code.get().hash_code();
                 }
               };

      private: struct EqualTo {
                 bool operator()(TypeInfoRef lhs, TypeInfoRef rhs) const
                 {
                   return lhs.get() == rhs.get();
                 }
               };

      private: std::unordered_map<TypeInfoRef,
                                  std::unique_ptr<ignition::common::Event>,
                                  Hasher, EqualTo> events;
    };
    }
  }
}

#endif  // IGNITION_GAZEBO_EVENTMANAGER_HH_
