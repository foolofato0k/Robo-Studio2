#ifndef UR3_CONTROL_HPP_
#define UR3_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <vector>
#include <string>



class UR3Control : public rclcpp::Node
{
public:
    UR3Control();

private:
    // ROS2 Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_init_;

    // Moveit Interfaces
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    const std::string planning_group_ = "ur_manipulator";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Internal state
    std::vector<geometry_msgs::msg::Pose> original_waypoints_;      // Robot Control Points
    std::vector<geometry_msgs::msg::Pose> updated_waypoints_; 
    std::atomic<bool> control_in_progress_;                         // Robot movement in progress

    // Member functions
    void goalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void timerCallback();
    void delayedInit();
    bool planAndExecuteCartesianPath(moveit::planning_interface::MoveGroupInterface& move_group,
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        rclcpp::Logger logger);

    // std::vector<moveit_msgs::msg::CollisionObject> createCollisionObjects(
    //     const std::vector<geometry_msgs::msg::Pose>& waypoints,
    //     const std::string& planning_frame);
    std::vector<geometry_msgs::msg::Pose> updateWaypointsForDrawing(
        const std::vector<geometry_msgs::msg::Pose>& original_waypoints, double offset_z);
};

#endif  // UR3_CONTROL_HPP_