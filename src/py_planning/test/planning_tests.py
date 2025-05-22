import pytest
from geometry_msgs.msg import Pose
from py_planning.pose_sequence_builder import PoseSequenceBuilder

def create_pose(x, y, z=0.0):
    """
    Helper function to create a Pose with specified x, y, and optional z.
    
    Ensures all position fields are explicitly cast to float, as required by ROS.
    """
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    return pose


def test_single_line_pose_sequence():
    """
    Test PoseSequenceBuilder.build_pose_array with a simple two-point stroke.

    Verifies that:
    - The correct number of poses are returned (center, lift → draw → draw → lift).
    - The Z values correctly reflect lifting and drawing heights.
    """
    lift_height = 0.2
    draw_height = 0.05

    # Create a single stroke with two points
    stroke = [
        create_pose(0.0, 0.0),
        create_pose(0.1, 0.1)
    ]
    stroke_list = [stroke]

    pose_array = PoseSequenceBuilder.build_pose_array(
        scaled_strokes=stroke_list,
        lift_height=lift_height,
        draw_height=draw_height
    )

    # Expect 5 poses: center, lifted start, drawing start, drawing end, lifted end
    assert len(pose_array.poses) == 5

    # Pose 0: center
    assert pose_array.poses[0].position.z == pytest.approx(0.2)

    # Pose 1: lifted start
    assert pose_array.poses[1].position.z == pytest.approx(lift_height)

    # Pose 2: drawing start
    assert pose_array.poses[2].position.z == pytest.approx(draw_height)

    # Pose 3: drawing end
    assert pose_array.poses[3].position.z == pytest.approx(draw_height)

    # Pose 4: lifted end
    assert pose_array.poses[4].position.z == pytest.approx(lift_height)

    
def test_pose_scaling():
    """
    Test PoseSequenceBuilder.scale_and_center by scaling a square of poses into an A4 landscape page.

    Verifies that:
    - All scaled poses remain within the page's width and height bounds.
    """
    # Original pose list (forming a square from 0.0 to 0.1)
    original_stroke = [
        (0.0, 0.0),
        (0.1, 0.0),
        (0.1, 0.1),
        (0.0, 0.1)
    ]
    strokes = [original_stroke]

    # A4 dimensions in meters (landscape orientation)
    target_width = 0.297
    target_height = 0.21

    # Use PoseSequenceBuilder's static method for scaling
    scaled_strokes = PoseSequenceBuilder.scale_and_center(
        strokes,
        target_width=target_width,
        target_height=target_height,
        center=(0.0, 0.0)
    )

    # Flatten the list of scaled poses
    scaled_poses = [pose for stroke in scaled_strokes for pose in stroke]

    # Check all scaled poses fit within page bounds
    for pose in scaled_poses:
        x, y = pose.position.x, pose.position.y
        assert -target_width / 2 <= x <= target_width / 2, f"x={x} out of bounds"
        assert -target_height / 2 <= y <= target_height / 2, f"y={y} out of bounds"

def main():
    # Run pytest on the current module (i.e., this file)
    pytest.main([__file__])


if __name__ == "__main__":
    main()