
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf/model.h>
#include <fstream>
#include <array>
#include <memory>
#include <thread>

class PositionPublisher : public rclcpp::Node {
public:
    PositionPublisher(const KDL::Vector& tcp_goal)
        : Node("position_publisher"), tcp_goal_(tcp_goal) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_position_controller/commands", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PositionPublisher::publish_positions, this));

        // Hardcoded UR type
        std::string ur_type = "ur5";
        std::string urdf_xacro_file = ament_index_cpp::get_package_share_directory("ur_description") + "/urdf/ur.urdf.xacro";

        // Generate URDF from Xacro with required arguments
        std::string urdf_content = generate_urdf(urdf_xacro_file, ur_type);

        // Parse URDF and generate KDL tree
        if (!kdl_parser::treeFromString(urdf_content, kdl_tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return;
        }

        // Generate kinematic chain
        if (!kdl_tree_.getChain("base_link", "wrist_3_link", kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain");
            return;
        }

        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
        ik_solver_vel_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(kdl_chain_, *fk_solver_, *ik_solver_vel_);

        RCLCPP_INFO(this->get_logger(), "URDF and KDL chain successfully loaded");
    }

private:
    void publish_positions() {
        // Example start joint positions
        KDL::JntArray jnt_pos_start(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
            jnt_pos_start(i) = 0.0;
        }

        // Compute forward kinematics to get the current TCP position
        KDL::Frame tcp_pos_start;
        fk_solver_->JntToCart(jnt_pos_start, tcp_pos_start);
        KDL::Vector tcp_start = tcp_pos_start.p;

        // Generate linear path
        int steps = 100;  // Number of waypoints in the path
        std::vector<KDL::Vector> path = generate_linear_path(tcp_start, tcp_goal_, steps);

        for (const auto& waypoint : path) {
            // Set goal TCP position for the current waypoint
            KDL::Frame tcp_pos_goal(tcp_pos_start.M, waypoint);

            // Compute inverse kinematics to get the goal joint positions
            KDL::JntArray jnt_pos_goal(kdl_chain_.getNrOfJoints());
            if (ik_solver_->CartToJnt(jnt_pos_start, tcp_pos_goal, jnt_pos_goal) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to compute IK for waypoint");
                return;
            }

            // Publish the goal joint positions
            std_msgs::msg::Float64MultiArray msg;
            for (unsigned int i = 0; i < jnt_pos_goal.rows(); ++i) {
                msg.data.push_back(jnt_pos_goal(i));
            }
            publisher_->publish(msg);

            // Optional: Add a small delay to allow the robot to move to the next waypoint
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Update start joint positions for the next iteration
            jnt_pos_start = jnt_pos_goal;
        }
    }

    std::string generate_urdf(const std::string &xacro_file, const std::string &ur_type) {
        std::string command = "ros2 run xacro xacro " + xacro_file +
                              " ur_type:=" + ur_type +
                              " name:=ur" +
                              " safety_limits:=false" +
                              " safety_pos_margin:=0.15" +
                              " safety_k_position:=20";
        std::array<char, 128> buffer;
        std::string result;
        std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }

    std::vector<KDL::Vector> generate_linear_path(const KDL::Vector& start, const KDL::Vector& goal, int steps) {
        std::vector<KDL::Vector> path;
        for (int i = 0; i <= steps; ++i) {
            double alpha = static_cast<double>(i) / steps;
            KDL::Vector waypoint = start * (1.0 - alpha) + goal * alpha;
            path.push_back(waypoint);
        }
        return path;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver_;
    KDL::Vector tcp_goal_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    if (argc != 4) {
        std::cerr << "Usage: position_publisher <x> <y> <z>" << std::endl;
        return 1;
    }

    double x = std::stod(argv[1]);
    double y = std::stod(argv[2]);
    double z = std::stod(argv[3]);

    KDL::Vector tcp_goal(x, y, z);
    auto node = std::make_shared<PositionPublisher>(tcp_goal);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
