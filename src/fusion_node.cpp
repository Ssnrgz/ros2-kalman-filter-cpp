#include <memory>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node") {
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SensorFusionNode::odom_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));
        
        // Publisher
        filtered_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/filtered_pose", 10);

        // Kalman Filter Initialization
        // State Vector [x, y, vx, vy]
        x = Eigen::VectorXd(4); 
        x << 0, 0, 0, 0;

        // Matrices
        P = Eigen::MatrixXd::Identity(4, 4) * 1.0;  // Covariance
        F = Eigen::MatrixXd::Identity(4, 4);        // State Transition
        H = Eigen::MatrixXd::Zero(2, 4);            // Measurement Matrix
        H(0,0) = 1; H(1,1) = 1;                     // Measuring x and y

        R = Eigen::MatrixXd::Identity(2, 2) * 0.1;  // Measurement Noise
        Q = Eigen::MatrixXd::Identity(4, 4) * 0.01; // Process Noise

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node Initialized");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 1. PREDICT STEP
        double dt = 0.03; // Approx time step (30Hz)
        F(0, 2) = dt;
        F(1, 3) = dt;
        
        x = F * x;
        P = F * P * F.transpose() + Q;

        // 2. UPDATE STEP (Using Odom Position)
        Eigen::VectorXd z(2);
        z << msg->pose.pose.position.x, msg->pose.pose.position.y;

        Eigen::VectorXd y = z - H * x;
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;

        publish_filtered_pose(msg->header.stamp, msg->pose.pose.orientation);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Correct velocity using IMU acceleration (Simple integration)
        double dt = 0.01;
        x[2] += msg->linear_acceleration.x * dt; 
        x[3] += msg->linear_acceleration.y * dt;
    }

    void publish_filtered_pose(const rclcpp::Time & stamp, const geometry_msgs::msg::Quaternion & q) { 
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.position.x = x[0];
        pose_msg.pose.position.y = x[1];
        pose_msg.pose.orientation = q; 
        filtered_pose_pub_->publish(pose_msg);
    }

    Eigen::VectorXd x; 
    Eigen::MatrixXd P, F, H, R, Q;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}