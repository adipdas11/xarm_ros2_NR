#include "xarm_planner/xarm_planner.h"

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[combined_xarm_planner] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("combined_xarm_planner", node_options);
    RCLCPP_INFO(node->get_logger(), "combined_xarm_planner start");

    signal(SIGINT, exit_sig_handler);

    int dof;
    node->get_parameter_or("dof", dof, 5);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string group_name = robot_type + std::to_string(dof);
    std::string gripper_group_name = robot_type + "_gripper";
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));

    if (prefix != "") {
        group_name = prefix + group_name;
        gripper_group_name = prefix + gripper_group_name;
    }

    RCLCPP_INFO(node->get_logger(), "namespace: %s, arm group_name: %s, gripper group_name: %s", node->get_namespace(), group_name.c_str(), gripper_group_name.c_str());

    // Initialize planners for both the arm and gripper
    xarm_planner::XArmPlanner arm_planner(node, group_name);
    xarm_planner::XArmPlanner gripper_planner(node, gripper_group_name);

    // // Define the target poses for the arm
    // geometry_msgs::msg::Pose target_pose1;
    // target_pose1.position.x = 0.3;
    // target_pose1.position.y = -0.1;
    // target_pose1.position.z = 0.2;
    // target_pose1.orientation.x = 1;
    // target_pose1.orientation.y = 0;
    // target_pose1.orientation.z = 0;
    // target_pose1.orientation.w = 0;

    // geometry_msgs::msg::Pose target_pose2;
    // target_pose2.position.x = 0.3;
    // target_pose2.position.y = 0.1;
    // target_pose2.position.z = 0.2;
    // target_pose2.orientation.x = 1;
    // target_pose2.orientation.y = 0;
    // target_pose2.orientation.z = 0;
    // target_pose2.orientation.w = 0;

    // pick coordinate
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.position.x = 0.4514419734477997;
    target_pose1.position.y = -0.22518490254878998;
    target_pose1.position.z = 0.09468356519937515;
    target_pose1.orientation.x = 0.9931374788284302;
    target_pose1.orientation.y = -0.11641029268503189;
    target_pose1.orientation.z = 0.010585245676338673;
    target_pose1.orientation.w = 0.00382005563005805;

    // up coordinate
    geometry_msgs::msg::Pose target_pose2;
    target_pose2.position.x = 0.407879114151001;
    target_pose2.position.y = -0.12362045794725418;
    target_pose2.position.z = 0.4031376540660858;
    target_pose2.orientation.x = 0.9918055534362793;
    target_pose2.orientation.y = -0.1277429312467575;
    target_pose2.orientation.z = -0.001832111389376223;
    target_pose2.orientation.w = -0.0003035841218661517;

    // drop coordinate
    geometry_msgs::msg::Pose target_pose3;
    target_pose3.position.x = 0.5999776721000671;
    target_pose3.position.y = 0.17626743018627167;
    target_pose3.position.z = 0.09468356519937515;
    target_pose3.orientation.x = 0.9928913116455078;
    target_pose3.orientation.y = -0.1186826154589653;
    target_pose3.orientation.z = 0.008288935758173466;
    target_pose3.orientation.w = -0.0035467171110212803;

    // Define joint positions for the gripper
    std::vector<double> open_gripper(6, 0.0);  // Open
    std::vector<double> close_gripper(6, 0.85); // Close

    while (rclcpp::ok())
    {
        // Move to the first pose
        arm_planner.planPoseTarget(target_pose1);
        arm_planner.executePath();

        gripper_planner.planJointTarget(open_gripper);
        gripper_planner.executePath();

        // Close the gripper
        gripper_planner.planJointTarget(close_gripper);
        gripper_planner.executePath();

        // Move to the second pose
        arm_planner.planPoseTarget(target_pose2);
        arm_planner.executePath();

        arm_planner.planPoseTarget(target_pose3);
        arm_planner.executePath();

        // Open the gripper
        gripper_planner.planJointTarget(open_gripper);
        gripper_planner.executePath();

        gripper_planner.planJointTarget(close_gripper);
        gripper_planner.executePath();

        arm_planner.planPoseTarget(target_pose2);
        arm_planner.executePath();
    }

    RCLCPP_INFO(node->get_logger(), "combined_xarm_planner over");
    return 0;
}
