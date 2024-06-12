#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

class MoveItCartesianPlanner : public rclcpp::Node
{
public:
    MoveItCartesianPlanner()
        : Node("moveit_cartesian_planner"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(*tf_buffer_)
    {
        std::string urdf_file_path = "/home/om/ros2_ws/src/ur_description/urdf/ur5.urdf";
        robot_description_ = readURDFFile(urdf_file_path);

        if (robot_description_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read the URDF file.");
            rclcpp::shutdown();
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Options options("manipulator", robot_description_);
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), options, tf_buffer_, rclcpp::Duration::from_seconds(10.0));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_position_controller/commands", 10);
    }

    void plan_cartesian_path(const std::vector<geometry_msgs::msg::Pose> &waypoints)
    {
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction == 1.0)
        {
            for (const auto &point : trajectory.joint_trajectory.points)
            {
                std_msgs::msg::Float64MultiArray msg;
                for (double position : point.positions)
                {
                    msg.data.push_back(position);
                    std::cout << position << " ";
                }
                publisher_->publish(msg);
                std::cout << std::endl;
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cartesian path planning failed.");
        }
    }

    geometry_msgs::msg::Pose get_current_pose()
    {
        return move_group_->getCurrentPose().pose;
    }

private:

    std::string readURDFFile(const std::string &file_path)
    {
        std::ifstream urdf_file(file_path);
        if (!urdf_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", file_path.c_str());
            return "";
        }

        std::stringstream urdf_content;
        urdf_content << urdf_file.rdbuf();
        urdf_file.close();
        return urdf_content.str();
    }

    std::string robot_description_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

std::vector<geometry_msgs::msg::Pose> generate_linear_waypoints(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &end_pose, int num_waypoints)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 0; i <= num_waypoints; ++i)
    {
        double fraction = static_cast<double>(i) / num_waypoints;
        geometry_msgs::msg::Pose pose;
        pose.position.x = start_pose.position.x + fraction * (end_pose.position.x - start_pose.position.x);
        pose.position.y = start_pose.position.y + fraction * (end_pose.position.y - start_pose.position.y);
        pose.position.z = start_pose.position.z + fraction * (end_pose.position.z - start_pose.position.z);
        pose.orientation = start_pose.orientation;
        waypoints.push_back(pose);
    }
    return waypoints;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItCartesianPlanner>();

    geometry_msgs::msg::Pose start_pose = node->get_current_pose();

    geometry_msgs::msg::Pose goal_pose;
    std::cout << "Enter goal pose (x y z qx qy qz qw): ";
    std::cin >> goal_pose.position.x >> goal_pose.position.y >> goal_pose.position.z
             >> goal_pose.orientation.x >> goal_pose.orientation.y >> goal_pose.orientation.z >> goal_pose.orientation.w;

    auto waypoints = generate_linear_waypoints(start_pose, goal_pose, 10);
    node->plan_cartesian_path(waypoints);

    rclcpp::shutdown();
    return 0;
}
