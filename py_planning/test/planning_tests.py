import pytest
from geometry_msgs.msg import Pose
from py_planning.pose_planner_node import PoseSequenceBuilder, PoseScaler

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
    Test PoseSequenceBuilder with a simple two-point stroke.

    Verifies that:
    - The correct number of poses are returned (lift → draw → draw → lift).
    - The Z values correctly reflect lifting and drawing heights.
    """
    builder = PoseSequenceBuilder(lift_height=0.2, draw_height=0.05)

    # Create a fake stroke with 2 points
    pose1 = create_pose(0, 0)
    pose2 = create_pose(0.1, 0.1)

    builder.add_sequence([pose1, pose2])
    result = builder.build_pose_array()

    # Expect 4 poses: lifted start, drawing start, drawing end, lifted end
    assert len(result) == 4

    # Check Z positions for lifting and drawing
    assert result[0].position.z == pytest.approx(0.2)  # lifted start
    assert result[1].position.z == pytest.approx(0.05)  # lowered (draw) start
    assert result[3].position.z == pytest.approx(0.2)  # lifted end


def test_pose_scaling():
    """
    Test PoseScaler by scaling a square of poses into an A4 landscape page.

    Verifies that:
    - All scaled poses remain within the page's width and height bounds.
    """
    # Original pose list (forming a square from 0.0 to 0.1)
    original_poses = [
        create_pose(0.0, 0.0),
        create_pose(0.1, 0.0),
        create_pose(0.1, 0.1),
        create_pose(0.0, 0.1)
    ]

    # A4 dimensions in meters (landscape orientation)
    target_width = 0.297
    target_height = 0.21

    # Initialize scaler centered at origin
    scaler = PoseScaler(target_width, target_height, center=(0.0, 0.0))
    scaled_poses = scaler.scale_and_center(original_poses)

    # Check all scaled poses fit within page bounds
    for pose in scaled_poses:
        x, y = pose.position.x, pose.position.y
        assert -target_width / 2 <= x <= target_width / 2
        assert -target_height / 2 <= y <= target_height / 2
