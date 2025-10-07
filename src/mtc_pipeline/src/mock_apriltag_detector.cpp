#include <rclcpp/rclcpp.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <map>

class MockAprilTagDetector : public rclcpp::Node
{
public:
    MockAprilTagDetector() : Node("mock_apriltag_detector")
    {
        // Fixed tag positions on a virtual table
        tag_positions_[0] = {0.5, 0.0, 0.02};   // Tag 0: center, 50cm forward
        tag_positions_[1] = {0.4, 0.2, 0.02};   // Tag 1: left side
        tag_positions_[2] = {0.4, -0.2, 0.02};  // Tag 2: right side

        // Publishers
        detection_pub_ = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/apriltag/detections", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer for publishing at 10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockAprilTagDetector::publish_detections, this));

        RCLCPP_INFO(this->get_logger(), "Mock AprilTag Detector started - simulating 3 tags");
    }

private:
    void publish_detections()
    {
        auto msg = apriltag_msgs::msg::AprilTagDetectionArray();
        msg.header.stamp = this->now();
        msg.header.frame_id = "zivid_optical_frame";

        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        for (const auto& [tag_id, pos] : tag_positions_) {
            // Create detection message
            apriltag_msgs::msg::AprilTagDetection detection;
            detection.id = tag_id;
            detection.family = "tag36h11";
            detection.hamming = 0;

            msg.detections.push_back(detection);

            // Create TF transform
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = msg.header.stamp;
            transform.header.frame_id = "base_link";
            transform.child_frame_id = "tag36h11:" + std::to_string(tag_id);

            transform.transform.translation.x = pos[0];
            transform.transform.translation.y = pos[1];
            transform.transform.translation.z = pos[2];

            // Flat orientation (tag lying on table)
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            transforms.push_back(transform);
        }

        detection_pub_->publish(msg);
        tf_broadcaster_->sendTransform(transforms);
    }

    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<int, std::array<double, 3>> tag_positions_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockAprilTagDetector>());
    rclcpp::shutdown();
    return 0;
}
