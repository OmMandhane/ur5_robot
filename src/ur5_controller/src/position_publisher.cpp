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

class PositionPublisher : public rclcpp::Node {
public:
    PositionPublisher() 
        : Node("position_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/simple_position_controller/commands", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/simple_position_controller/commands",
            10,
            std::bind(&PositionPublisher::joint_callback, this, std::placeholders::_1));

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
    void joint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Directly publish received joint angles to the position controller
        std_msgs::msg::Float64MultiArray joint_positions_msg;
        joint_positions_msg.data = msg->data; // Copy the joint angles
        publisher_->publish(joint_positions_msg);

        // Print received joint angles
        for (auto position : msg->data) {
            std::cout << position << " ";
        }
        std::cout << std::endl;
    }

    std::string generate_urdf(const std::string &xacro_file, const std::string &ur_type) {
        std::string command = "ros2 run xacro xacro " + xacro_file +
                              " ur_type:=" + ur_type +
                              " name:=ur" +  // Ensure 'name' argument is passed
                              " safety_limits:=false" +  // Default to false or add other required arguments
                              " safety_pos_margin:=0.15" +  // Default values
                              " safety_k_position:=20";  // Default values
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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PositionPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
