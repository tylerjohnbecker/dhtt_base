#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "dhtt_msgs/srv/experiment_command.hpp"

class ExperimentMoveIt : public rclcpp::Node
{
public:
    ExperimentMoveIt(rclcpp::Node::SharedPtr& moveItInterfaceNode,
                     std::string nodeName = "Experiment_MoveIt",
                     const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                     bool executePlan = false, bool collisionObjects = true);
    ~ExperimentMoveIt();

private:
    /*Private Member Functions*/

    /**
     * Add collision objects for the ridgeback base and gantry behind the arm
     */
    void setCollisionObjects();

    /**
     * Clear the collision objects we added
     */
    void clearCollisionObjects();

    /**
     * Create a plan to that target pose, which should already be set by planExecuteToPose()
     */
    bool _planExecute();

    /**
     * Plan and execute to a pose
     */
    bool planExecuteToPose(geometry_msgs::msg::Pose target_pose);

    /**
     * Get pose from xyz
     * default manipulator orientation is straight down
     */
    const geometry_msgs::msg::Pose getPoseFromXYZ(double x, double y, double z,
                                                  double wRot = MANIPULATORROTW,
                                                  double xRot = MANIPULATORROTX,
                                                  double yRot = MANIPULATORROTY,
                                                  double zRot = MANIPULATORROTZ);

    /**
     * Move the arm to a position where it can pick an object at xyz
     *
     * - Accounts for the vertical size of the manipulator (assumes vertical orientation)
     * - Moves preHeight above target, then lowers straight down to target
     */
    bool moveToPick(double x, double y, double z);
    /**
     * Overload moveToPick with TransformStamped instead of xyz
     *
     * Move the arm to a position where it can pick an object at xyz
     *
     * - Accounts for the vertical size of the manipulator (assumes vertical orientation)
     * - Moves preHeight above target, then lowers straight down to target
     */
    bool moveToPick(geometry_msgs::msg::TransformStamped x);

    bool moveToHome();

    bool moveToDepot();

    void setTolerance(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                      double jointTolerance, double goalTolerance, double orientTolerance,
                      double planTime, unsigned int planAttempts);

    geometry_msgs::msg::TransformStamped getBlockPose(std::string targetFrame,
                                                      std::string fromFrame);

    std::map<std::string, geometry_msgs::msg::TransformStamped> detectBlocks();

    // gripper poition should be between 0 and 0.8 radians (46 degrees)
    bool setGripperPosition(double rad);
    bool openGripper();
    bool closeGripper();

    bool movePickMoveDepot(geometry_msgs::msg::TransformStamped blockPose);

    std::map<std::string, geometry_msgs::msg::TransformStamped>
    scanBlocks();

    void experimentCommand(const std::shared_ptr<dhtt_msgs::srv::ExperimentCommand::Request> request,
                           std::shared_ptr<dhtt_msgs::srv::ExperimentCommand::Response> response);

    /* Private Member Variables */

    // map of block names to their poses relative world
    std::map<std::string, geometry_msgs::msg::TransformStamped> blockPoses;

    // for debugging, prevent execution
    const bool EXECUTEPLAN;

    const std::vector<std::string> blockNames = {
        "redSmall", "redMedium", "redLarge",
        "greenSmall", "greenMedium", "greenLarge",
        "blueSmall", "blueMedium", "blueLarge",
        "yellowSmall", "yellowMedium", "yellowLarge"
    };

    std::vector<std::string> collisionObjectIDs;

    /* ROS Services */

    // TF listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Listen to experiment commands
    rclcpp::Service<dhtt_msgs::srv::ExperimentCommand>::SharedPtr commandService;

    // MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface move_groupManipulator;
    moveit::planning_interface::MoveGroupInterface move_groupGripper;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // default initialized

    /* CONSTANTS */
    // manipulator pose helpers
    static constexpr double MANIPULATORROTW = 1e-6; // 0
    static constexpr double MANIPULATORROTX = 0.707000;
    static constexpr double MANIPULATORROTY = 0.707000;
    static constexpr double MANIPULATORROTZ = 1e-6; // 0
    /* for 45 degree nose down
    wxyz
    0.2705;
    0.6532;
    0.6532;
    0.2705;
    */
    static constexpr double MANIPULATORVERTICALOFFSET = 0.125;
};
