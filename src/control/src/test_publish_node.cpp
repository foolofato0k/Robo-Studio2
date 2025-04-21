#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PoseArrayPublisher : public rclcpp::Node
{
public:
    PoseArrayPublisher() : Node("pose_array_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("goals_pose_topic", 10);

        // Create a timer to publish once after startup (or you can use a loop if you want multiple publishes)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&PoseArrayPublisher::publishPoses, this));
    }

private: 
    void publishPoses()
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = "world";  // Use your robot's planning frame
        
        // Create a few example poses
 	    geometry_msgs::msg::Pose target_pose1;
 	    target_pose1.position.x = 0.1;
 	    target_pose1.position.y = 0.4;
 	    target_pose1.position.z = 0.05;
 	    target_pose1.orientation.w = 1.0;  // Orientation is not used for the cube objects.
 	    pose_array.poses.push_back(target_pose1);
        
        geometry_msgs::msg::Pose target_pose2;
        target_pose2.position.x = 0.35;
        target_pose2.position.y = -0.2;
        target_pose2.position.z = 0.05;
        target_pose2.orientation.w = 1.0;
        pose_array.poses.push_back(target_pose2);
        
        geometry_msgs::msg::Pose target_pose3;
        target_pose3.position.x = 0.35;
        target_pose3.position.y = 0.2;
        target_pose3.position.z = 0.05;
        target_pose3.orientation.w = 1.0;
        pose_array.poses.push_back(target_pose3);

        publisher_->publish(pose_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu poses", pose_array.poses.size());

        timer_->cancel();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseArrayPublisher>());
    rclcpp::shutdown();
    return 0;
}