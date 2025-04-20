import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool

class PoseArrayPublisher(Node):

    def __init__(self):
        super().__init__('test_pose_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'goals_pose_topic', 10)
        self.subscriber_ = self.create_subscription(Bool, 'photo_confirmed', self.photo_confirmed_callback)
        
        self.photo_confirmed = False
        # Create a timer that fires once after 1 second
        self.timer_ = self.create_timer(1.0, self.publish_poses)

    def photo_confirmed_callback(self, msg: Bool):
        self.photo_confirmed = msg.data
        self.get_logger().info(f"Photo confirmed: {self.photo_confirmed}")

    def publish_poses(self):
        if(self.photo_confirmed):
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'world'

            # Create some example poses
            pose1 = Pose()
            pose1.position.x = 0.1
            pose1.position.y = 0.4
            pose1.position.z = 0.05
            pose1.orientation.w = 1.0

            pose2 = Pose()
            pose2.position.x = 0.35
            pose2.position.y = -0.2
            pose2.position.z = 0.05
            pose2.orientation.w = 1.0

            pose3 = Pose()
            pose3.position.x = 0.35
            pose3.position.y = 0.2
            pose3.position.z = 0.05
            pose3.orientation.w = 1.0

            pose_array.poses.extend([pose1, pose2, pose3])

            self.publisher_.publish(pose_array)
            self.get_logger().info(f'Published {len(pose_array.poses)} poses')

            # Cancel the timer after publishing once
            self.timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()