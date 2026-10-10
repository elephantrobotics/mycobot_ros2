#include <gtest/gtest.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_loader.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace
{
const char* const URDF = R"(
<robot name="start_state_test">
  <link name="base"/><link name="arm"/><link name="finger"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="arm"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="gripper_controller" type="revolute">
    <parent link="arm"/><child link="finger"/><axis xyz="0 1 0"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>)";

class CurrentStartTest : public testing::Test
{
protected:
  pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter> loader_{
    "moveit_core", "planning_request_adapter::PlanningRequestAdapter"};
  planning_request_adapter::PlanningRequestAdapterPtr adapter_;
  planning_scene::PlanningScenePtr scene_;

  void SetUp() override
  {
    adapter_ = loader_.createSharedInstance("pro450_gazebo/UseCurrentStartState");
    auto urdf = urdf::parseURDF(URDF);
    auto srdf = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf->initString(*urdf, "<robot name='start_state_test'><group name='arm'><joint name='joint1'/></group></robot>"));
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model);
  }
};

TEST_F(CurrentStartTest, EachRequestUsesNewSceneStateRatherThanCachedLaunchState)
{
  planning_interface::MotionPlanRequest request;
  request.start_state.joint_state.name = {"joint1", "gripper_controller"};
  request.start_state.joint_state.position = {0.0, 0.0};
  request.group_name = "arm";
  request.allowed_planning_time = 3.0;
  request.max_velocity_scaling_factor = 0.5;
  request.goal_constraints.resize(1);
  request.goal_constraints[0].joint_constraints.resize(1);
  request.goal_constraints[0].joint_constraints[0].joint_name = "joint1";
  request.goal_constraints[0].joint_constraints[0].position = 0.8;
  scene_->getAllowedCollisionMatrixNonConst().setEntry("base", "arm", true);
  for (double current : {0.3, -0.4})
  {
    scene_->getCurrentStateNonConst().setVariablePosition("joint1", current);
    scene_->getCurrentStateNonConst().setVariablePosition("gripper_controller", 0.7);
    bool called = false;
    planning_request_adapter::PlanningRequestAdapter::PlannerFn planner =
      [&](const planning_scene::PlanningSceneConstPtr& scene,
          const planning_interface::MotionPlanRequest& adapted,
          planning_interface::MotionPlanResponse&) {
        called = true;
        EXPECT_EQ(scene.get(), scene_.get());
        EXPECT_TRUE(adapted.start_state.is_diff);
        EXPECT_TRUE(adapted.start_state.joint_state.name.empty());
        const auto state = scene->getCurrentStateUpdated(adapted.start_state);
        EXPECT_DOUBLE_EQ(state->getVariablePosition("joint1"), current);
        EXPECT_DOUBLE_EQ(state->getVariablePosition("gripper_controller"), 0.7);
        EXPECT_EQ(adapted.goal_constraints, request.goal_constraints);
        EXPECT_EQ(adapted.group_name, request.group_name);
        EXPECT_EQ(adapted.allowed_planning_time, request.allowed_planning_time);
        EXPECT_EQ(adapted.max_velocity_scaling_factor, request.max_velocity_scaling_factor);
        collision_detection::AllowedCollision::Type collision_allowed;
        EXPECT_TRUE(scene->getAllowedCollisionMatrix().getEntry("base", "arm", collision_allowed));
        EXPECT_EQ(collision_allowed, collision_detection::AllowedCollision::ALWAYS);
        return true;
      };
    planning_interface::MotionPlanResponse response;
    std::vector<std::size_t> added;
    EXPECT_TRUE(adapter_->adaptAndPlan(planner, scene_, request, response, added));
    EXPECT_TRUE(called);
    EXPECT_TRUE(added.empty());
    EXPECT_EQ(request.start_state.joint_state.position, (std::vector<double>{0.0, 0.0}));
  }
}

TEST_F(CurrentStartTest, DoesNotConvertPlannerFailureIntoSuccess)
{
  planning_request_adapter::PlanningRequestAdapter::PlannerFn planner =
    [](const planning_scene::PlanningSceneConstPtr&, const planning_interface::MotionPlanRequest&,
       planning_interface::MotionPlanResponse&) { return false; };
  planning_interface::MotionPlanResponse response;
  std::vector<std::size_t> added;
  EXPECT_FALSE(adapter_->adaptAndPlan(planner, scene_, planning_interface::MotionPlanRequest(), response, added));
}
}  // namespace
