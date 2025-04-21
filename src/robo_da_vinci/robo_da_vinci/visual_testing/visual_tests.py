
from geometry_msgs.msg import Pose
from py_planning.pose_planner_node import PoseSequenceBuilder
from py_planning.visual_utils import plot_2d_points, plot_3d_points

def test_single_line_visualisation():
    builder = PoseSequenceBuilder(lift_height=0.2, draw_height=0.05)

    input_data = [
        (0.1, 0.4),
        (0.35, -0.2),
        (0.35, 0.2)
    ]

    poses = []
    for x, y in input_data:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        poses.append(pose)


    builder.add_sequence(poses)
    result = builder.build_pose_array()
    print("2D input data")
    plot_2d_points(input_data)
    print("3D Planned Tajectory")
    plot_3d_points(result)
    print(len(result))

def test_multi_line_visualisation():
    builder = PoseSequenceBuilder(lift_height=0.2, draw_height=0.05)

    input_data1 = [
        (0.1, 0.4),
        (0.35, -0.2),
        (0.35, 0.2)
    ]
    
    poses = []
    for x, y in input_data1:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        poses.append(pose)

    builder.add_sequence(poses)


    input_data2 = [
        (0.3, 0.4),
        (0.55, -0.2),
        (0.55, 0.2)
    ]

    poses = []
    for x, y in input_data2:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        poses.append(pose)

    builder.add_sequence(poses)
    result = builder.build_pose_array()

    combined_input_data = input_data1 + input_data2

    print("2D input data")
    plot_2d_points(combined_input_data)
    print("3D Planned Tajectory")
    plot_3d_points(result)
    print(len(result))


def main():
    print("TEST 1 Easy Points Test")
    test_single_line_visualisation()

    print("TEST 2 Harder Points Test")
    test_multi_line_visualisation()

if __name__ == '__main__':
    main()