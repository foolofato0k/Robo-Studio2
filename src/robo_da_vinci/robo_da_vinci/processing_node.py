from py_planning.pose_sequence_builder import PoseSequenceBuilder
from robo_da_vinci.img_processor import WebcamImg, detectFaceEdges, getPaths, tesselate
from py_planning.visual_utils import plot_3d_points
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
import cv2 as cv
import os


class ProcessingNode(Node):

    def __init__(self):
        super().__init__('path_planning')
        self.publisher_ = self.create_publisher(PoseArray, 'goals_pose_topic', 10)
        self.subscriber_ = self.create_subscription(Bool, 'photo_confirmed', self.photo_confirmed_callback, 10)
        
        # Create a timer that fires once after 1 second
        self.timer_ = self.create_timer(1.0, self.main_loop)

        # Loop variables
        self.photo_confirmed = True

    def photo_confirmed_callback(self, msg: Bool):
        self.photo_confirmed = msg.data
        self.get_logger().info(f"Photo confirmed: {self.photo_confirmed}")

    def main_loop(self):
        # IF Take Photo
        # THEN take and process photo
        # THEN then plan based on photo

        if self.photo_confirmed:
            image = None
            webcam = False

            # TAKE PHOTO
            if webcam:
                image = WebcamImg(image)
                if image == None:
                    exit()
            else:
                script_dir = os.path.dirname(os.path.realpath(__file__))
                image_path = os.path.join(script_dir, 'test_images', 'webcam_img.jpg')
                image = cv.imread(image_path)

            if image is None:
                print("Failed to load image. Check the path: test_images/webcam_img.jpg")
                exit()

            # PROCESSING EDGES ________________________________________
            poster = detectFaceEdges(image)
            paths = getPaths(poster)
            image_paths = tesselate(paths)

            # CONSTRUCT PATH PLANNER
            pose_builder = PoseSequenceBuilder(image_paths)

            # SCALE TO A4 SIZE AND CENTER AT (0, 0.25)
            pose_builder.scale_and_center(0.297,0.210,(0.1,0.4))

            ####### TSP IN THE FUTURE TO-DO!!! #######

            # PLAN PATH
            stroke_plan = pose_builder.build_pose_array(0.15, 0.1)
            
            # Plot poses in 3D
            if(True):
                plot_3d_points(stroke_plan.poses)

            # Keep only the first 10 poses TESTING PURPOSES
            #stroke_plan.poses = stroke_plan.poses[:500]

            pose_builder.print_pose_array(stroke_plan)
            # PUBLISH PATH
            self.publisher_.publish(stroke_plan)
            self.get_logger().info(f'Published {len(stroke_plan._poses)} strokes')


            self.photo_confirmed = False




def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()