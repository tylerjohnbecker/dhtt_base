#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "dhtt_msgs/srv/experiment_command.hpp"

#include "dhtt_reorder/experiment_moveit.hpp"

ExperimentMoveIt::ExperimentMoveIt(rclcpp::Node::SharedPtr &moveItInterfaceNode,
                                   std::string nodeName,
                                   const rclcpp::NodeOptions &options,
                                   bool executePlan, bool collisionObjects)
    : rclcpp::Node(nodeName, options),
      EXECUTEPLAN(executePlan),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      commandService(this->create_service<dhtt_msgs::srv::ExperimentCommand>(
          "ExperimentCommand",
          std::bind(
              &ExperimentMoveIt::experimentCommand,
              this, std::placeholders::_1, std::placeholders::_2))),
      move_groupManipulator(moveItInterfaceNode, "manipulator"),
      move_groupGripper(moveItInterfaceNode, "gripper")
{
    const auto &LOGGER = this->get_logger();

    if (collisionObjects)
    {
        this->setCollisionObjects();
    }

    this->move_groupManipulator.setMaxVelocityScalingFactor(0.7);
    this->move_groupManipulator.setPlanningPipelineId(
        "pilz_industrial_motion_planner");
    this->move_groupManipulator.setPlannerId("PTP");

    // Default tolerance is 0.000100, 0.000100, 0.001000. That's 0.1mm and 0.001rad
    // setTolerance(this->move_groupManipulator, 0.000100, 0.000100, 0.050000, 5, 10);

    RCLCPP_INFO(LOGGER, "Running with EXECUTEPLAN = %s",
                this->EXECUTEPLAN ? "True" : "False");
    RCLCPP_INFO(LOGGER, "Current Planner ID %s::%s",
                this->move_groupManipulator.getPlanningPipelineId().c_str(),
                this->move_groupManipulator.getPlannerId().c_str());
    RCLCPP_INFO(LOGGER, "Tolerance: GoalJoint %f, GoalPos %f, GoalOrient %f",
                this->move_groupManipulator.getGoalJointTolerance(),
                this->move_groupManipulator.getGoalPositionTolerance(),
                this->move_groupManipulator.getGoalOrientationTolerance());

    RCLCPP_INFO(LOGGER, "%s Ready to Go!", nodeName.c_str());
}

ExperimentMoveIt::~ExperimentMoveIt()
{
    // TODO
    this->clearCollisionObjects();
}

void ExperimentMoveIt::setCollisionObjects()
{
    auto &move_group_interface = this->move_groupManipulator;
    auto &planning_scene_interface = this->planning_scene_interface;
    const auto &LOGGER = this->get_logger();

    std::vector<std::string> currentObjects = planning_scene_interface.
        getKnownObjectNames();
    if (std::find(currentObjects.begin(), currentObjects.end(), "box1") !=
        currentObjects.end())
    {
        // already added it
        return;
    }

    // collisions
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive base;
    base.type = base.BOX;
    base.dimensions.resize(3);
    base.dimensions[base.BOX_X] = 2.0;
    base.dimensions[base.BOX_Y] = 2.0;
    base.dimensions[base.BOX_Z] = 0.5;
    geometry_msgs::msg::Pose base_pose;
    base_pose.orientation.w = 1.0;
    base_pose.position.x = 0.0;
    base_pose.position.y = 0.0;
    base_pose.position.z = -0.26;

    shape_msgs::msg::SolidPrimitive gantry;
    gantry.type = gantry.BOX;
    gantry.dimensions.resize(3);
    gantry.dimensions[gantry.BOX_X] = 1.0;
    gantry.dimensions[gantry.BOX_Y] = 1.0;
    gantry.dimensions[gantry.BOX_Z] = 0.61;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.37 - 0.5;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.61 / 2.0;

    collision_object.primitives.push_back(base);
    collision_object.primitives.push_back(gantry);
    collision_object.primitive_poses.push_back(base_pose);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    for (auto x : collision_objects)
    {
        this->collisionObjectIDs.push_back(x.id);
    }
}

void ExperimentMoveIt::clearCollisionObjects()
{
    // TODO
    return;
}

bool ExperimentMoveIt::_planExecute()
{
    auto &move_group_interface = this->move_groupManipulator;
    const auto &LOGGER = this->get_logger();

    // iife: https://en.wikipedia.org/wiki/Immediately_invoked_function_expression
    auto const [planSuccess, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (planSuccess)
    {
        if (this->EXECUTEPLAN)
        {
            return static_cast<bool>(move_group_interface.execute(plan));
        }

        RCLCPP_INFO(LOGGER, "Would execute");
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }

    return planSuccess;
}

bool ExperimentMoveIt::planExecuteToPose(geometry_msgs::msg::Pose target_pose)
{
    auto &move_group_interface = this->move_groupManipulator;
    const auto &LOGGER = this->get_logger();

    RCLCPP_INFO(LOGGER, "Planning to [%f, %f, %f]",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);
    move_group_interface.setPoseTarget(target_pose);
    return _planExecute();
}

const geometry_msgs::msg::Pose ExperimentMoveIt::
getPoseFromXYZ(double x, double y, double z,
               double wRot, double xRot, double yRot, double zRot)
{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = wRot;
    msg.orientation.x = xRot;
    msg.orientation.y = yRot;
    msg.orientation.z = zRot;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

bool ExperimentMoveIt::moveToPick(double x, double y, double z)
{
    z += this->MANIPULATORVERTICALOFFSET;
    const double preHeight = 0.03;
    bool success;

    geometry_msgs::msg::Pose prePose = getPoseFromXYZ(x, y, z + preHeight);
    geometry_msgs::msg::Pose postPose = getPoseFromXYZ(x, y, z);

    success = planExecuteToPose(prePose);
    if (not success)
    {
        return false;
    }

    success = planExecuteToPose(postPose);

    if (not success)
    {
        return false;
    }

    return true;
}

bool ExperimentMoveIt::moveToPick(geometry_msgs::msg::TransformStamped x)
{
    return moveToPick(x.transform.translation.x,
                      x.transform.translation.y, x.transform.translation.z);
}

bool ExperimentMoveIt::moveToHome()
{
    geometry_msgs::msg::Pose pose = getPoseFromXYZ(0.45, 0.0, 0.60);
    return planExecuteToPose(pose);
}

bool ExperimentMoveIt::moveToDepot()
{
    geometry_msgs::msg::Pose pose = getPoseFromXYZ(0.2, 0.0, 0.4);
    return planExecuteToPose(pose);
}

void ExperimentMoveIt::setTolerance(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    double jointTolerance, double goalTolerance, double orientTolerance,
    double planTime, unsigned int planAttempts)
{
    move_group_interface.setGoalJointTolerance(goalTolerance);
    move_group_interface.setGoalJointTolerance(jointTolerance);
    move_group_interface.setGoalOrientationTolerance(orientTolerance);
    move_group_interface.setPlanningTime(planTime);
    move_group_interface.setNumPlanningAttempts(planAttempts);
}

geometry_msgs::msg::TransformStamped ExperimentMoveIt::
getBlockPose(std::string targetFrame, std::string fromFrame)
{
    auto &tf_buffer_ = this->tf_buffer_;
    const auto &LOGGER = this->get_logger();
    geometry_msgs::msg::TransformStamped t;

    try
    {
        t = tf_buffer_->lookupTransform(
            targetFrame, fromFrame,
            tf2::TimePointZero);
        return t;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO(
            LOGGER, "Could not transform %s to %s: %s",
            targetFrame.c_str(), fromFrame.c_str(), ex.what());
        throw ex;
    }
}

std::map<std::string, geometry_msgs::msg::TransformStamped> ExperimentMoveIt::
detectBlocks()
{
    auto &blockNames = this->blockNames;
    const auto &LOGGER = this->get_logger();

    // map of block positions
    std::map<std::string, geometry_msgs::msg::TransformStamped> blocks;
    for (std::string x : blockNames)
    {
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = getBlockPose("world", x);
            blocks[x] = t;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(LOGGER, "Could not detect block %s, skipping",
                        x.c_str());
        }
    }

    for (auto const &x : blocks)
    {
        RCLCPP_INFO(LOGGER, "Found %s at XYZ: %f %f %f", x.first.c_str(),
                    x.second.transform.translation.x,
                    x.second.transform.translation.y,
                    x.second.transform.translation.z);
    }

    return blocks;
}

bool ExperimentMoveIt::setGripperPosition(double rad)
{
    const auto &LOGGER = this->get_logger();
    auto &move_group_interface = this->move_groupGripper;

    auto const joints = [&]
    {
        sensor_msgs::msg::JointState j;
        j.header.frame_id = "robotiq_85_base_link";
        j.name = {"robotiq_85_left_knuckle_joint"};
        j.position = {rad};
        // j.effort = {0.1};
        // j.velocity = {0.1};
        return j;
    }();

    RCLCPP_INFO(LOGGER, "Setting gripper to %f rads", rad);

    bool const jointSuccess = move_group_interface.setJointValueTarget(joints);
    if (not jointSuccess)
    {
        RCLCPP_ERROR(LOGGER, "Failed to set gripper joint target");
        return false;
    }

    auto const [planSuccess, plan] = [&move_group_interface]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();
    if (not planSuccess)
    {
        RCLCPP_ERROR(LOGGER, "Failed to plan gripper");
        return false;
    }

    if (this->EXECUTEPLAN)
    {
        const bool executeSuccess = static_cast<bool>(move_group_interface.
            execute(plan));
        if (not executeSuccess)
        {
            RCLCPP_ERROR(LOGGER, "Failed to execute gripper");
            return false;
        }
    }
    else
    {
        RCLCPP_INFO(LOGGER, "Would execute gripper");
    }

    return true;
}

bool ExperimentMoveIt::openGripper()
{
    return setGripperPosition(0.1);
}

bool ExperimentMoveIt::closeGripper()
{
    return setGripperPosition(0.7);
}

bool ExperimentMoveIt::movePickMoveDepot(
    geometry_msgs::msg::TransformStamped blockPose)
{
    const auto &LOGGER = this->get_logger();
    bool go;

    go = openGripper();
    if (go)
        go = moveToPick(blockPose);
    if (go)
        go = closeGripper();
    if (go)
        go = moveToDepot();
    if (go)
        go = openGripper();

    if (not go)
    {
        RCLCPP_ERROR(LOGGER, "movePickMoveDepot failed");
    }

    return go;
}

/**
 * Scan for blocks and save them this class;
 */
std::map<std::string, geometry_msgs::msg::TransformStamped>
ExperimentMoveIt::scanBlocks()
{
    const auto &LOGGER = this->get_logger();

    auto const leftPose = getPoseFromXYZ(0.45, 0.24, 0.37);
    auto const middlePose = getPoseFromXYZ(0.45, 0.0, 0.37);
    auto const rightPose = getPoseFromXYZ(0.45, -0.24, 0.37);
    std::vector<geometry_msgs::msg::Pose> const poses{
        leftPose, middlePose, rightPose};
    std::map<std::string, geometry_msgs::msg::TransformStamped> blocks;

    for (auto pose : poses)
    {
        bool go = planExecuteToPose(pose);
        if (not go)
        {
            RCLCPP_ERROR(LOGGER, "scanBlocks movement failed");
            break;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(2500));

        auto localBlocks = detectBlocks();
        // will only append new blocks, already detected ones are kept same
        // cpp17 adds std::merge which has more functionality
        blocks.insert(localBlocks.begin(), localBlocks.end());
    }

    this->blockPoses = blocks;
    return blocks; // may be empty
}

void ExperimentMoveIt::experimentCommand(
    const std::shared_ptr<dhtt_msgs::srv::ExperimentCommand::Request> request,
    std::shared_ptr<dhtt_msgs::srv::ExperimentCommand::Response> response)

{
    const auto &LOGGER = this->get_logger();

    if (request->cmd.empty())
    {
        RCLCPP_ERROR(LOGGER, "Experiment command cannot be empty");
        response->success = false;
        return;
    }

    // movePickMoveDepot some block
    if (request->cmd == request->PICKPLACE)
    {
        const auto blockPoseIt = blockPoses.find(request->arg);

        if (request->arg.empty())
        {
            RCLCPP_ERROR(LOGGER, "Experiment arg cannot be empty");
            response->success = false;
            return;
        }
        else if (this->blockPoses.empty())
        {
            RCLCPP_ERROR(
                LOGGER,
                "No block poses known. Did you forget to cmd:ScanBlocks?");
            response->success = false;
            return;
        }
        else if (blockPoseIt == blockPoses.end())
        {
            RCLCPP_ERROR(LOGGER, "%s was not found, can't pick.",
                         request->arg.c_str());
            response->success = false;
            return;
        }
        else // the block exists, pick it
        {
            RCLCPP_INFO(LOGGER, "Got good command to pick %s",
                        blockPoseIt->first.c_str());
            response->success = this->movePickMoveDepot(blockPoseIt->second);
            return;
        }
    }

    // Scan across the workspace for blocks
    else if (request->cmd == request->SCANBLOCKS)
    {
        RCLCPP_INFO(LOGGER, "Got good command to ScanBlocks");
        this->scanBlocks();
        response->success = true;
        return;
    }

    // Move to home position
    else if (request->cmd == request->MOVEHOME)
    {
        RCLCPP_INFO(LOGGER, "Got good command to MoveHome");
        response->success = this->moveToHome();
        return;
    }

    // Open Gripper
    else if (request->cmd == request->OPENGRIPPER)
    {
        RCLCPP_INFO(LOGGER, "Got good command to OpenGripper");
        response->success = this->openGripper();
        return;
    }

    // Close Gripper
    else if (request->cmd == request->CLOSEGRIPPER)
    {
        RCLCPP_INFO(LOGGER, "Got good command to CloseGripper");
        response->success = this->closeGripper();
        return;
    }

    return;
}

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto moveItInterface = std::make_shared<rclcpp::Node>(
        "Experiment_MoveIt_moveItInterface",
        rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true));

    const auto experimentNode = std::make_shared<ExperimentMoveIt>(
        moveItInterface,
        "Experiment_MoveIt",
        rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true),
        true);

    executor.add_node(moveItInterface);
    executor.add_node(experimentNode);

    executor.spin();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}