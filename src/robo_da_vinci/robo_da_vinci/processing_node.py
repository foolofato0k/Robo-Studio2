import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from py_planning.pose_sequence_builder import PoseSequenceBuilder
import img_processor.img_processor as im_process

class Processing(Node):

    def __init__(self):
        super().__init__('path_planning')
        self.publisher_ = self.create_publisher(PoseArray, 'goals_pose_topic', 10)

        
        # Create a timer that fires once after 1 second
        #self.timer_ = self.create_timer(1.0, self.publish_poses)

    def publish_poses(self, poses):
        i = None

def main(args=None):
    rclpy.init(args=args)
    node = Processing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()