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
#ifndef IGNITION_GAZEBO_COMPONENTS_COMPONENT_HH_
#define IGNITION_GAZEBO_COMPONENTS_COMPONENT_HH_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  // Forward declarations.
  template<typename DataType> class ComponentPrivate;

  /// \brief Convenient type to be used by components that don't wrap any data.
  /// I.e. they act as tags and their presence is enough to infer something
  /// about the entity.
  using NoData = std::add_lvalue_reference<void>;

  /// \brief Base class for all components.
  class BaseComponent
  {
    /// \brief Default constructor.
    public: BaseComponent() = default;

    /// \brief Default destructor.
    public: virtual ~BaseComponent() = default;
  };

  /// \brief A component type that wraps any data type. The intention is for
  /// this class to be used to create simple components while avoiding a lot of
  /// boilerplate code. The Identifier must be a unique type so that type
  /// aliases can be used to create new components. However the type does not
  /// need to be defined anywhere
  /// eg.
  ///     using Static = Component<bool, class StaticTag>;
  ///
  /// Note, however, that this scheme does not have a mechanism to stop someone
  /// accidentally defining another component that wraps a bool as such:
  ///     using AnotherComp = Component<bool, class StaticTag>;
  /// In this case, Static and AnotherComp are exactly the same types and would
  /// not be differentiable by the EntityComponentManager.
  template <typename DataType, typename Identifier>
  class Component: public BaseComponent
  {
    /// \brief Default constructor
    public: explicit Component() = default;

    /// \brief Constructor
    /// \param[in] _simpleWrapper Component to copy
    public: explicit Component(const DataType &_data);

    /// \brief Copy Constructor
    /// \param[in] _simpleWrapper Component component to copy.
    public: Component(const Component &_simpleWrapper);

    /// \brief Move Constructor
    /// \param[in] _simpleWrapper Component component to move.
    public: Component(Component &&_simpleWrapper) noexcept = default;

    /// \brief Destructor.
    public: ~Component() override = default;

    /// \brief Move assignment operator.
    /// \param[in] _simpleWrapper Component component to move.
    /// \return Reference to this.
    public: Component &operator=(
                Component &&_simpleWrapper) noexcept = default;

    /// \brief Copy assignment operator.
    /// \param[in] _simpleWrapper Component component to copy.
    /// \return Reference to this.
    public: Component &operator=(const Component &_simpleWrapper);

    /// \brief Equality operator.
    /// \param[in] _simpleWrapper Component to compare to.
    /// \return True if equal.
    public: bool operator==(const Component &_simpleWrapper) const;

    /// \brief Inequality operator.
    /// \param[in] _simpleWrapper Component to compare to.
    /// \return True if different.
    public: bool operator!=(const Component &_simpleWrapper) const;

    /// \brief Get the simpleWrapper data.
    /// \return The actual simpleWrapper information.
    public: const DataType &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ComponentPrivate<DataType>> dataPtr;

    /// \brief Component name.
    public: inline static std::string name{""};

    /// \brief Component id.
    public: inline static uint64_t id{0};
  };

  /// \brief Specialization for components that don't wrap any data.
  /// This class to be used to create simple components that represent
  /// just a "tag", while avoiding a lot of boilerplate code. The Identifier
  /// must be a unique type so that type aliases can be used to create new
  /// components. However the type does not need to be defined anywhere eg.
  ///
  ///     using Joint = Component<NoData, class JointTag>;
  ///
  template <typename Identifier>
  class Component<NoData, Identifier> : public BaseComponent
  {
    // Documentation inherited
    public: bool operator==(const Component<NoData, Identifier> &) const;

    // Documentation inherited
    public: bool operator!=(const Component<NoData, Identifier> &) const;

    /// \brief Component name.
    public: inline static std::string name{""};

    /// \brief Component id.
    public: inline static uint64_t id{0};
  };

  template <typename DataType>
  class ComponentPrivate
  {
    /// \brief Constructor.
    /// \param[in] _simpleWrapper Component data.
    public: explicit ComponentPrivate(DataType _data)
            : data(std::move(_data))
    {
    }

    /// \brief The data being wrapped.
    public: DataType data;
  };

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier>::Component(const DataType &_data)
    : dataPtr(std::make_unique<ComponentPrivate<DataType>>(_data))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier>::Component(
      const Component<DataType, Identifier> &_simpleWrapper)
      : dataPtr(std::make_unique<ComponentPrivate<DataType>>(
            _simpleWrapper.Data()))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  const DataType &Component<DataType, Identifier>::Data() const
  {
    return this->dataPtr->data;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier> &Component<DataType, Identifier>::
  operator=(const Component<DataType, Identifier> &_simpleWrapper)
  {
    this->dataPtr->data = _simpleWrapper.Data();
    return *this;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  bool Component<DataType, Identifier>::
  operator==(const Component<DataType, Identifier> &_simpleWrapper) const
  {
    return this->dataPtr->data == _simpleWrapper.Data();
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  bool Component<DataType, Identifier>::
  operator!=(const Component<DataType, Identifier> &_simpleWrapper) const
  {
    return this->dataPtr->data != _simpleWrapper.Data();
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  bool Component<NoData, Identifier>::operator==(
      const Component<NoData, Identifier> &) const
  {
    return true;
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  bool Component<NoData, Identifier>::operator!=(
      const Component<NoData, Identifier> &) const
  {
    return false;
  }
}
}
}
}
#endif
