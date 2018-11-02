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

#include <gtest/gtest.h>
#include <ignition/common/Console.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>

#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

class SimulationRunnerTest : public ::testing::TestWithParam<int>
{
};

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateEntities)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  std::vector<SystemPluginPtr> systems;
  SimulationRunner runner(root.WorldByIndex(0), systems);

  // Check component types
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::World>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Model>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Link>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Collision>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Visual>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Name>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::ParentEntity>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Geometry>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Material>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Inertial>()));

  // Check entities
  // 1 x world + 3 x model + 3 x link + 3 x collision + 3 x visual
  EXPECT_EQ(13u, runner.EntityCompMgr().EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  EntityId worldEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::World,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::World *_world,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _world);
      EXPECT_NE(nullptr, _name);

      EXPECT_EQ("default", _name->Data());

      worldCount++;

      worldEntity = _entity;
      return true;
    });

  EXPECT_EQ(1u, worldCount);
  EXPECT_NE(kNullEntity, worldEntity);

  // Check models
  unsigned int modelCount{0};
  EntityId boxModelEntity = kNullEntity;
  EntityId cylModelEntity = kNullEntity;
  EntityId sphModelEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::Model,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Model *_model,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _model);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      modelCount++;

      EXPECT_EQ(worldEntity, _parent->Id());
      if (modelCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("box", _name->Data());
        boxModelEntity = _entity;
      }
      else if (modelCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(-1, -2, -3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("cylinder", _name->Data());
        cylModelEntity = _entity;
      }
      else if (modelCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("sphere", _name->Data());
        sphModelEntity = _entity;
      }
      return true;
    });

  EXPECT_EQ(3u, modelCount);
  EXPECT_NE(kNullEntity, boxModelEntity);
  EXPECT_NE(kNullEntity, cylModelEntity);
  EXPECT_NE(kNullEntity, sphModelEntity);

  // Check links
  unsigned int linkCount{0};
  EntityId boxLinkEntity = kNullEntity;
  EntityId cylLinkEntity = kNullEntity;
  EntityId sphLinkEntity = kNullEntity;
  runner.EntityCompMgr().Each<components::Link,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Link *_link,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _link);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      linkCount++;

      if (linkCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_link", _name->Data());
        EXPECT_EQ(boxModelEntity, _parent->Id());
        boxLinkEntity = _entity;
      }
      else if (linkCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_link", _name->Data());
        EXPECT_EQ(cylModelEntity, _parent->Id());
        cylLinkEntity = _entity;
      }
      else if (linkCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_link", _name->Data());
        EXPECT_EQ(sphModelEntity, _parent->Id());
        sphLinkEntity = _entity;
      }
      return true;
    });

  EXPECT_EQ(3u, linkCount);
  EXPECT_NE(kNullEntity, boxLinkEntity);
  EXPECT_NE(kNullEntity, cylLinkEntity);
  EXPECT_NE(kNullEntity, sphLinkEntity);

  // Check inertials
  unsigned int inertialCount{0};
  runner.EntityCompMgr().Each<components::Link, components::Inertial>(
    [&](const EntityId & _entity,
        const components::Link *_link,
        const components::Inertial *_inertial)->bool
    {
      EXPECT_NE(nullptr, _link);
      EXPECT_NE(nullptr, _inertial);

      inertialCount++;

      if (_entity == boxLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(1.0, math::Vector3d(1.0, 1.0, 1.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      else if (_entity == cylLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(2.0, math::Vector3d(2.0, 2.0, 2.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      else if (_entity == sphLinkEntity)
      {
        EXPECT_EQ(math::MassMatrix3d(3.0, math::Vector3d(3.0, 3.0, 3.0),
                                     math::Vector3d::Zero),
                  _inertial->Data().MassMatrix());
      }
      return true;
    });

  EXPECT_EQ(3u, inertialCount);

  // Check collisions
  unsigned int collisionCount{0};
  runner.EntityCompMgr().Each<components::Collision,
                            components::Geometry,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &/*_entity*/,
        const components::Collision *_collision,
        const components::Geometry *_geometry,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _collision);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      collisionCount++;

      if (collisionCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.11, 0.11, 0.11, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_collision", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(3, 4, 5),
                  _geometry->Data().BoxShape()->Size());
      }
      else if (collisionCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.21, 0.21, 0.21, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_collision", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(0.2, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(0.1, _geometry->Data().CylinderShape()->Length());
      }
      else if (collisionCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.31, 0.31, 0.31, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_collision", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(23.4, _geometry->Data().SphereShape()->Radius());
      }
      return true;
    });

  EXPECT_EQ(3u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  runner.EntityCompMgr().Each<components::Visual,
                            components::Geometry,
                            components::Material,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &/*_entity*/,
        const components::Visual *_visual,
        const components::Geometry *_geometry,
        const components::Material *_material,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)->bool
    {
      EXPECT_NE(nullptr, _visual);
      EXPECT_NE(nullptr, _geometry);
      EXPECT_NE(nullptr, _material);
      EXPECT_NE(nullptr, _pose);
      EXPECT_NE(nullptr, _parent);
      EXPECT_NE(nullptr, _name);

      visualCount++;

      if (visualCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.12, 0.12, 0.12, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("box_visual", _name->Data());

        EXPECT_EQ(boxLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::BOX, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().BoxShape());
        EXPECT_EQ(math::Vector3d(1, 2, 3),
                  _geometry->Data().BoxShape()->Size());

        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(1, 0, 0), _material->Data().Specular());
      }
      else if (visualCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.22, 0.22, 0.22, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("cylinder_visual", _name->Data());

        EXPECT_EQ(cylLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::CYLINDER, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().CylinderShape());
        EXPECT_DOUBLE_EQ(2.1, _geometry->Data().CylinderShape()->Radius());
        EXPECT_DOUBLE_EQ(10.2, _geometry->Data().CylinderShape()->Length());

        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 1, 0), _material->Data().Specular());
      }
      else if (visualCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.32, 0.32, 0.32, 0, 0, 0),
            _pose->Data());

        EXPECT_EQ("sphere_visual", _name->Data());

        EXPECT_EQ(sphLinkEntity, _parent->Id());

        EXPECT_EQ(sdf::GeometryType::SPHERE, _geometry->Data().Type());
        EXPECT_NE(nullptr, _geometry->Data().SphereShape());
        EXPECT_DOUBLE_EQ(1.2, _geometry->Data().SphereShape()->Radius());

        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Emissive());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Ambient());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Diffuse());
        EXPECT_EQ(math::Color(0, 0, 1), _material->Data().Specular());
      }
      return true;
    });

  EXPECT_EQ(3u, visualCount);
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateJointEntities)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/demo_joint_types.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  std::vector<SystemPluginPtr> systems;
  SimulationRunner runner(root.WorldByIndex(0), systems);

  // Check component types
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::World>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Joint>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::JointAxis>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::JointType>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::ChildLinkName>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::ParentLinkName>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::ParentEntity>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Pose>()));
  EXPECT_TRUE(runner.EntityCompMgr().HasComponentType(
      EntityComponentManager::ComponentType<components::Name>()));

  const sdf::Model *model = root.WorldByIndex(0)->ModelByIndex(1);

  auto testAxis = [](const std::string &_jointName,
                      const sdf::JointAxis *_jointAxis,
                      const auto *_axisComp)
  {
    ASSERT_TRUE(nullptr != _axisComp) << "Axis not found for " << _jointName;
    EXPECT_EQ(_jointAxis->Xyz(), _axisComp->Data().Xyz());
    EXPECT_DOUBLE_EQ(_jointAxis->Lower(), _axisComp->Data().Lower());
    EXPECT_DOUBLE_EQ(_jointAxis->Upper(), _axisComp->Data().Upper());
    EXPECT_DOUBLE_EQ(_jointAxis->Effort(), _axisComp->Data().Effort());
  };

  auto testJoint = [&testAxis](const sdf::Joint *_joint,
      const components::JointAxis *_axis,
      const components::JointAxis2 *_axis2,
      const components::ParentLinkName *_parentLinkName,
      const components::ChildLinkName *_childLinkName,
      const components::Pose *_pose,
      const components::Name *_name,
      bool _checkAxis = true,
      bool _checkAxis2 = false)
  {
    ASSERT_TRUE(nullptr != _joint);

    // Even if the pose element isn't explicitly set in the sdf, a default one
    // is created during parsing, so it's safe to compare here.
    EXPECT_EQ(_joint->Pose(), _pose->Data());

    if (_checkAxis)
      testAxis(_joint->Name(), _joint->Axis(0), _axis);

    if (_checkAxis2)
      testAxis(_joint->Name(), _joint->Axis(1), _axis2);

    EXPECT_EQ(_joint->ParentLinkName(), _parentLinkName->Data());
    EXPECT_EQ(_joint->ChildLinkName(), _childLinkName->Data());
    EXPECT_EQ(_joint->Name(), _name->Data());
  };

  std::set<std::string> jointsToCheck = {
    "revolute_demo",
    "gearbox_demo"
    "revolute2_demo",
    "prismatic_demo",
    "ball_demo",
    "screw_demo",
    "universal_demo",
    "prismatic_demo",
    "fixed_demo"
  };

  std::set<sdf::JointType> jointTypes;
  runner.EntityCompMgr().Each<components::Joint,
                            components::JointType,
                            components::ParentLinkName,
                            components::ChildLinkName,
                            components::Pose,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Joint * /*_joint*/,
        const components::JointType *_jointType,
        const components::ParentLinkName *_parentLinkName,
        const components::ChildLinkName *_childLinkName,
        const components::Pose *_pose,
        const components::Name *_name)->bool
    {
      jointTypes.insert(_jointType->Data());
      auto axis =
          runner.EntityCompMgr().Component<components::JointAxis>(_entity);
      auto axis2 =
          runner.EntityCompMgr().Component<components::JointAxis2>(_entity);

      const sdf::Joint *joint = model->JointByName(_name->Data());

      if (jointsToCheck.find(_name->Data()) != jointsToCheck.end())
      {
        bool checkAxis = ("fixed_demo" != _name->Data()) &&
                         ("ball_demo" != _name->Data());
        bool checkAxis2 = ("gearbox_demo" == _name->Data()) ||
                          ("revolute2_demo" == _name->Data()) ||
                          ("universal_demo" == _name->Data());
        testJoint(joint, axis, axis2, _parentLinkName, _childLinkName, _pose,
            _name, checkAxis, checkAxis2);
      }

      return true;
    });

  EXPECT_EQ(8u, jointTypes.size());
}

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, Time)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  std::vector<SystemPluginPtr> systems;
  SimulationRunner runner(root.WorldByIndex(0), systems);

  // Check state
  EXPECT_TRUE(runner.Paused());
  EXPECT_EQ(0u, runner.CurrentInfo().iterations);
  EXPECT_EQ(0ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(0ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(1ms, runner.StepSize());

  runner.SetPaused(false);

  // Run
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(100u, runner.CurrentInfo().iterations);
  EXPECT_EQ(100ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(1ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(1ms, runner.StepSize());

  // Change step size and run
  runner.SetStepSize(2ms);
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(200u, runner.CurrentInfo().iterations);
  EXPECT_EQ(300ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(2ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(2ms, runner.StepSize());

  // Set paused
  runner.SetPaused(true);
  EXPECT_TRUE(runner.Paused());
  runner.SetPaused(false);
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(200u, runner.CurrentInfo().iterations);
  EXPECT_EQ(300ms, runner.CurrentInfo().simTime);
  EXPECT_EQ(2ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(2ms, runner.StepSize());

  // Unpause and run
  runner.SetPaused(false);
  EXPECT_TRUE(runner.Run(100));

  // Check state
  EXPECT_FALSE(runner.Paused());
  EXPECT_EQ(300u, runner.CurrentInfo().iterations);
  EXPECT_EQ(500ms, runner.CurrentInfo().simTime)
    << runner.CurrentInfo().simTime.count();
  EXPECT_EQ(2ms, runner.CurrentInfo().dt);
  EXPECT_EQ(1ms, runner.UpdatePeriod());
  EXPECT_EQ(2ms, runner.StepSize());
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(ServerRepeat, SimulationRunnerTest,
    ::testing::Range(1, 2));
