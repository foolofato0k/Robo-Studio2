import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def plot_2d_points(points_2d):
    x, y = zip(*points_2d)
    plt.figure()
    plt.title("2D Points")
    plt.scatter(x, y, c='blue')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.plot(x, y, color='blue', linestyle='--', marker='o')
    plt.show()

def plot_3d_points(poses):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #x, y, z = zip(*points_3d)
    x = [p.position.x for p in poses]
    y = [p.position.y for p in poses]
    z = [p.position.z for p in poses]

    ax.scatter(x, y, z, c='red')
    ax.set_title("3D Points")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1,1,1])
    plt.tight_layout()
    ax.plot(x, y, z, color='red', linestyle='--', marker='o')  # line + points
    plt.show()

def main():
    # Example conversion
    points_2d = [(0, 0), (1, 2), (2, 1), (3, 3)]
    points_3d = [(x, y, 0.05) for x, y in points_2d]  # simple Z height

    plot_2d_points(points_2d)
    plot_3d_points(points_3d)

    #plt.show()

if __name__ == "__main__":
    main()