from geometry_msgs.msg import Pose, PoseArray
from typing import List, Tuple
from robo_da_vinci.lkh_utils import solve_stroke_order_py

class PoseSequenceBuilder:
    @staticmethod
    def optimise_path(strokes: List[List[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
        if not strokes:
            raise ValueError("No strokes to optimize.")

        stroke_endpoints = []
        for stroke in strokes:
            #if len(stroke) < 2:
            #    raise ValueError("Each stroke must have at least 2 points.")
            x1, y1 = stroke[0]
            x2, y2 = stroke[-1]
            stroke_endpoints.append((x1, y1, x2, y2))

        tsp_order = solve_stroke_order_py(stroke_endpoints)

        optimized_strokes = []
        for idx in tsp_order:
            stroke = strokes[abs(idx)]
            if idx < 0:
                stroke = stroke[::-1]
            optimized_strokes.append(stroke)

        return optimized_strokes

    @staticmethod
    def scale_and_center(
        strokes: List[List[Tuple[float, float]]],
        target_width: float,
        target_height: float,
        center: Tuple[float, float] = (0.0, 0.0),
        margin: float = 0.02
    ) -> List[List[Pose]]:
        if not strokes:
            return []

        target_width -= margin * 2
        target_height -= margin * 2

        all_points = [pt for stroke in strokes for pt in stroke]
        xs, ys = zip(*all_points)

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        width = max_x - min_x
        height = max_y - min_y

        scale_x = target_width / width if width > 0 else 1.0
        scale_y = target_height / height if height > 0 else 1.0
        scale = min(scale_x, scale_y)

        x_center = (min_x + max_x) / 2
        y_center = (min_y + max_y) / 2
        x_offset = center[0] - x_center * scale
        y_offset = center[1] - y_center * scale

        scaled_strokes = []
        for stroke in strokes:
            pose_stroke = []
            for x, y in stroke:
                pose = Pose()
                pose.position.x = x * scale + x_offset
                pose.position.y = y * scale + y_offset
                pose.orientation.w = 1.0
                pose_stroke.append(pose)
            scaled_strokes.append(pose_stroke)

        return scaled_strokes

    @staticmethod
    def build_pose_array(
        scaled_strokes: List[List[Pose]],
        lift_height: float = 0.2,
        draw_height: float = 0.2
    ) -> PoseArray:
        if not scaled_strokes or not isinstance(scaled_strokes[0][0], Pose):
            raise ValueError("Provide scaled strokes with Pose objects.")

        pose_array = PoseArray()

        all_points = [p for stroke in scaled_strokes for p in stroke]
        if all_points:
            mean_x = sum(p.position.x for p in all_points) / len(all_points)
            mean_y = sum(p.position.y for p in all_points) / len(all_points)

            center_pose = Pose()
            center_pose.position.x = mean_x
            center_pose.position.y = mean_y
            center_pose.position.z = 0.2
            center_pose.orientation.w = 1.0
            pose_array.poses.append(center_pose)

        for stroke in scaled_strokes:
            if not stroke:
                continue

            start = stroke[0]
            end = stroke[-1]

            lifted_start = Pose()
            lifted_start.position.x = start.position.x
            lifted_start.position.y = start.position.y
            lifted_start.position.z = lift_height
            lifted_start.orientation.w = 1.0
            pose_array.poses.append(lifted_start)

            for pose in stroke:
                pose.position.z = draw_height
                pose_array.poses.append(pose)

            lifted_end = Pose()
            lifted_end.position.x = end.position.x
            lifted_end.position.y = end.position.y
            lifted_end.position.z = lift_height
            lifted_end.orientation.w = 1.0
            pose_array.poses.append(lifted_end)

        return pose_array

    @staticmethod
    def print_pose_array(pose_array: PoseArray):
        print("=== Pose Sequence ===")
        for i, pose in enumerate(pose_array.poses):
            pos = pose.position
            print(f"Pose {i}:  Position -> x: {pos.x:.4f}, y: {pos.y:.4f}, z: {pos.z:.4f}")
        print("=====================")