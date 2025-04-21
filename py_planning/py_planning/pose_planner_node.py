import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from planning import PoseScaler, PoseSequenceBuilder


class PoseSequenceBuilder:
    def __init__(self, lift_height=0.15, draw_height=0.05):
        self.sequences = []  # list of list of Pose
        self.lift_height = lift_height
        self.draw_height = draw_height
        self.test = False

    def add_sequence(self, pose_list):
        self.sequences.append(pose_list)


    def build_pose_array(self):
        all_poses = []

        for seq in self.sequences:
            if not seq:
                continue

            start = seq[0]
            end = seq[-1]

            # Go to lifted start
            lifted_start = Pose()
            lifted_start.position.x = float(start.position.x)
            lifted_start.position.y = float(start.position.y)
            lifted_start.position.z = float(self.lift_height)
            lifted_start.orientation.w = 1.0
            all_poses.append(lifted_start)

            # # Lower to drawing height
            # lowered_start = Pose()
            # lowered_start.position.x = start.position.x
            # lowered_start.position.y = start.position.y
            # lowered_start.position.z = self.draw_height
            # lowered_start.orientation.w = 1.0
            # all_poses.append(lowered_start)

            # Draw path
            for pose in seq:
                pose.position.z = float(self.draw_height)
                pose.orientation.w = 1.0
                all_poses.append(pose)

            # Lift at end
            lifted_end = Pose()
            lifted_end.position.x = float(end.position.x)
            lifted_end.position.y = float(end.position.y)
            lifted_end.position.z = float(self.lift_height)
            lifted_end.orientation.w = 1.0
            all_poses.append(lifted_end)

        return all_poses 


class PoseScaler:
    def __init__(self, target_width, target_height, center=(0.0,0.0),margin=0.02):
        self.target_width = target_width - margin * 2
        self.target_height = target_height - margin * 2
        self.center = center

    def scale_and_center(self,poses):
        if not poses:
            return poses
        
        xs = [pose.position.x for pose in poses]
        ys = [pose.position.y for pose in poses]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)

        width = max_x - min_x
        height = max_y - min_y

        # Calculate scale factor
        
        if width > 0:
            scale_x = self.target_width / width
        else:
            scale_x = 1.0
        if height > 0:
            scale_y = self.target_height / height
        else:
            scale_y = 1.0

        # Scales to the largest dimension
        scale = min(scale_x, scale_y)

        x_offset = self.center[0] - ((min_x + max_x) / 2) * scale
        y_offset = self.center[1] - ((min_y + max_y) / 2) * scale

        # Scale and translate poses
        scaled_poses = []
        for pose in poses:
            new_pose = Pose()
            new_pose.position.x = float(pose.position.x * scale + x_offset)
            new_pose.position.y = float(pose.position.y * scale + y_offset)
            new_pose.position.z = float(pose.position.z)  # Preserve Z
            new_pose.orientation = pose.orientation  # Preserve orientation
            scaled_poses.append(new_pose)

        return scaled_poses


class PosePlanner(Node):
    def __init__(self, sequences=None):
        super().__init__('path_planning')
        self.publisher_ = self.create_publisher(PoseArray, 'goals_pose_topic', 10)
        self.builder = PoseSequenceBuilder()
        if sequences:
            for seq in sequences:
                self.builder.add_sequence(seq)
        else:
           self.get_logger().info(f'no sequence input, using example path')
           self.declare_example_paths() 
        self.timer_ = self.create_timer(1.0, self.publish_poses_once)

    def publish_poses_once(self):
        #self.declare_example_paths()
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'world'
        pose_array.poses = self.builder.build_pose_array()
        
        
        self.publisher_.publish(pose_array)
        self.get_logger().info(f'Published {len(pose_array.poses)} poses')

        self.timer_.cancel()  # publish only once

    def declare_example_paths(self):
        # Line 1
        line1 = []
        for x in [0.1, 0.15, 0.2, 0.25]:
            pose = Pose()
            pose.position.x = x
            pose.position.y = 0.2
            line1.append(pose)
        self.builder.add_sequence(line1)

        # Line 2
        line2 = []
        for y in [0.2, 0.15, 0.1]:
            pose = Pose()
            pose.position.x = 0.3
            pose.position.y = y
            line2.append(pose)
        self.builder.add_sequence(line2)


def create_pose(x, y, z=0.0, w=1.0):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = w
    return pose


def main(args=None):
    rclpy.init(args=args)
    
    TEST1 = False
    TEST2 = True
    sequences = None
    if(TEST1):
        # A square path (10cm x 10cm)
        square = [
            create_pose(0.1, 0.4, 0.05),
            create_pose(0.4, 0.2, 0.05),
            create_pose(0.4, 0.3, 0.05),
            create_pose(0.3, 0.3, 0.05),
            create_pose(0.3, 0.2, 0.05)  # Closing the square
        ]
        sequences = [square]

    if(TEST2):
        # A triangle path
        triangle = [
            create_pose(0.2, 0.2, 0.05),
            create_pose(0.25, 0.3, 0.05),
            create_pose(0.15, 0.3, 0.05),
            create_pose(0.2, 0.2, 0.05)  # Closing the triangle
        ]
        square = [
            create_pose(0.1, 0.4, 0.05),   # Top-left
            create_pose(0.25, 0.4, 0.05),  # Top-right
            create_pose(0.25, 0.25, 0.05), # Bottom-right
            create_pose(0.1, 0.25, 0.05),  # Bottom-left
            create_pose(0.1, 0.4, 0.05)    # Closing the square
]
        sequences = [square, triangle]

    
    node = PosePlanner(sequences)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()