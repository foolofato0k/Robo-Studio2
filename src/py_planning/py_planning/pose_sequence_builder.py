from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
from typing import List, Tuple

'''
Class to plan a UR3 trajectory
Input a list of strokes (list of tuples (x, y))
Scale and Center (optional)
Build pose array which will lift ur3 to a given height between strokes
returns a geometry_msgs.Posearray
'''
class PoseSequenceBuilder:
    def __init__(self, strokes: List[List[Tuple[float, float]]],):
        self.original_strokes = strokes  # list of list of (x,y)
        self.scaled_strokes = []
    
    def scale_and_center(self, target_width, target_height, center=(0.0,0.0),margin=0.02):
        target_width = target_width - margin * 2
        target_height = target_height - margin * 2
        
        # get all points in a single list
        all_points = []
        for stroke in self.original_strokes:
            for point in stroke:
                all_points.append(point)

        if len(all_points) == 0:
            return[]
        
        xs = []
        ys = []

        for point in all_points:
            xs.append(point[0])
            ys.append(point[1])

        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)

        width = max_x - min_x
        height = max_y - min_y

        # Calculate scale factor
        
        if width > 0:
            scale_x = target_width / width
        else:
            scale_x = 1.0
        if height > 0:
            scale_y = target_height / height
        else:
            scale_y = 1.0

        # Scales to the largest dimension
        scale = min(scale_x, scale_y)

        x_center = (min_x + max_x) / 2
        y_center = (min_y + max_y) / 2
        x_offset = center[0] - x_center * scale
        y_offset = center[1] - y_center * scale

        # Scale and translate poses
        scaled_strokes = []
        for stroke in self.original_strokes:
            pose_stroke = []
            for point in stroke:
                x = point[0]
                y = point[1]

                pose = Pose()
                pose.position.x = x * scale + x_offset
                pose.position.y = y * scale + y_offset
                # pose.position.z = draw_height  # this will be adjusted later
                pose.orientation.w = 1.0
                pose_stroke.append(pose)
            self.scaled_strokes.append(pose_stroke)
        return self.scaled_strokes
    


    def build_pose_array(self, lift_height=0.15, draw_height=0.05):
        final_plan = PoseArray()

        if not self.scaled_strokes or not isinstance(self.scaled_strokes[0][0], Pose):
            raise ValueError("Call scale_and_center() first to convert strokes to Pose objects.")

        # Add initial overhead center pose
        all_points = [pose for stroke in self.scaled_strokes for pose in stroke]
        if all_points:
            mean_x = sum(p.position.x for p in all_points) / len(all_points)
            mean_y = sum(p.position.y for p in all_points) / len(all_points)

            center_pose = Pose()
            center_pose.position.x = mean_x
            center_pose.position.y = mean_y
            center_pose.position.z = 0.2
            center_pose.orientation.w = 1.0
            final_plan._poses.append(center_pose)
            #final_plan.append(center_pose)

        # Adjust and plan all the strokes
        for stroke in self.scaled_strokes:
            if not stroke:
                continue

            start = stroke[0]
            end = stroke[-1]

            # Go to lifted start
            lifted_start = Pose()
            lifted_start.position.x = start.position.x
            lifted_start.position.y = start.position.y
            lifted_start.position.z = lift_height
            lifted_start.orientation.w = 1.0
            #final_plan.append(lifted_start)
            final_plan._poses.append(lifted_start)

            # Draw path
            for pose in stroke:
                pose.position.z = draw_height
                final_plan._poses.append(pose)

                #final_plan.append(pose)

            # Lift at end
            lifted_end = Pose()
            lifted_end.position.x = end.position.x
            lifted_end.position.y = end.position.y
            lifted_end.position.z = lift_height
            lifted_end.orientation.w = 1.0
            final_plan._poses.append(lifted_end)
            #final_plan.append(lifted_end)

        return final_plan 
    
    def print_pose_array(self, pose_array: PoseArray):
        print("=== Pose Sequence ===")
        for i, pose in enumerate(pose_array.poses):
            pos = pose.position
            orient = pose.orientation
            #print(f"Pose {i}:")
            print(f"Pose {i}:  Position -> x: {pos.x:.4f}, y: {pos.y:.4f}, z: {pos.z:.4f}")
            #print(f"  Orientation -> x: {orient.x:.4f}, y: {orient.y:.4f}, z: {orient.z:.4f}, w: {orient.w:.4f}")
        print("=====================")
 

    