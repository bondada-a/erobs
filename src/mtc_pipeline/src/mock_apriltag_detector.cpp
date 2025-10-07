#include <rclcpp/rclcpp.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <map>

class MockAprilTagDetector : public rclcpp::Node
{
public:
    MockAprilTagDetector() : Node("mock_apriltag_detector")
    {
        // Declare parameters
        this->declare_parameter<std::vector<int64_t>>("tag_ids", std::vector<int64_t>{0, 1, 2});
        this->declare_parameter<bool>("publish_moving_tags", false);
        this->declare_parameter<double>("movement_radius", 0.1);
        this->declare_parameter<double>("movement_speed", 0.5);
        this->declare_parameter<std::string>("camera_frame", "zivid_optical_frame");

        // Get parameters
        tag_ids_ = this->get_parameter("tag_ids").as_integer_array();
        publish_moving_tags_ = this->get_parameter("publish_moving_tags").as_bool();
        movement_radius_ = this->get_parameter("movement_radius").as_double();
        movement_speed_ = this->get_parameter("movement_speed").as_double();
        camera_frame_ = this->get_parameter("camera_frame").as_string();

        // Initialize tag positions (on a virtual table)
        // Tag 0: Center of workspace
        tag_positions_[0] = {0.5, 0.0, 0.02};  // 50cm forward, 2cm above table
        // Tag 1: Left side
        tag_positions_[1] = {0.4, 0.2, 0.02};  // 40cm forward, 20cm left
        // Tag 2: Right side
        tag_positions_[2] = {0.4, -0.2, 0.02}; // 40cm forward, 20cm right

        // Create publishers
        detection_pub_ = this->create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/apriltag/detections", 10);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MockAprilTagDetector::publish_detections, this));

        RCLCPP_INFO(this->get_logger(), "Mock AprilTag Detector started");
        RCLCPP_INFO(this->get_logger(), "Simulating %zu tags", tag_ids_.size());
        if (publish_moving_tags_) {
            RCLCPP_INFO(this->get_logger(), "Tags will move in circles with radius %.2fm", movement_radius_);
        }
    }

private:
    void publish_detections()
    {
        auto msg = apriltag_msgs::msg::AprilTagDetectionArray();
        msg.header.stamp = this->now();
        msg.header.frame_id = camera_frame_;

        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // Get current time for movement animation
        double time = this->now().seconds();

        for (auto tag_id : tag_ids_) {
            // Skip if position not defined
            if (tag_positions_.find(tag_id) == tag_positions_.end()) {
                continue;
            }

            // Create detection
            apriltag_msgs::msg::AprilTagDetection detection;
            detection.id = tag_id;
            detection.family = "tag36h11";
            detection.hamming = 0;
            detection.goodness = 1.0;
            detection.decision_margin = 100.0;

            // Set corners (simplified - just placeholders)
            for (int i = 0; i < 4; ++i) {
                apriltag_msgs::msg::Point corner;
                corner.x = 100.0 + i * 10;
                corner.y = 100.0 + i * 10;
                detection.corners[i] = corner;
            }

            // Center point in image
            detection.centre.x = 320.0;  // Center of 640x480 image
            detection.centre.y = 240.0;

            msg.detections.push_back(detection);

            // Create and publish transform
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = msg.header.stamp;
            transform.header.frame_id = "base_link";  // Publish in base_link frame

            // Tag frame name format: family:id
            transform.child_frame_id = "tag36h11:" + std::to_string(tag_id);

            // Get base position
            auto pos = tag_positions_[tag_id];

            // Add movement if enabled
            if (publish_moving_tags_) {
                double angle = time * movement_speed_;
                pos[0] += movement_radius_ * std::cos(angle + tag_id * 2.0);  // Phase shift per tag
                pos[1] += movement_radius_ * std::sin(angle + tag_id * 2.0);
            }

            transform.transform.translation.x = pos[0];
            transform.transform.translation.y = pos[1];
            transform.transform.translation.z = pos[2];

            // Set orientation (facing up for table-top tags)
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);  // No rotation - tag is flat on table
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            transforms.push_back(transform);
        }

        // Publish detection array
        detection_pub_->publish(msg);

        // Publish TF transforms
        tf_broadcaster_->sendTransform(transforms);

        // Log occasionally
        static int counter = 0;
        if (++counter % 50 == 0) {  // Every 5 seconds at 10Hz
            RCLCPP_INFO(this->get_logger(), "Publishing %zu simulated tag detections", msg.detections.size());
        }
    }

    // Member variables
    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int64_t> tag_ids_;
    std::map<int, std::array<double, 3>> tag_positions_;
    bool publish_moving_tags_;
    double movement_radius_;
    double movement_speed_;
    std::string camera_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MockAprilTagDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}