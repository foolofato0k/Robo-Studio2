import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def input_coordinates():
    coordinates = []
    while True:
        user_input = input("Enter a coordinate [x, y] or type 'done' to finish: ")
        if user_input.lower() == 'done':
            break
        try:
            coordinate = list(map(float, user_input.strip("[]").split(",")))
            if len(coordinate) == 2:
                coordinates.append(coordinate)
            else:
                print("Invalid coordinate. Please enter coordinates as [x, y].")
        except ValueError:
            print("Invalid input. Please enter the coordinate as [x,y] (e.g., [2,3]).")
    return np.array(coordinates)

def scale_and_center_coordinates(coordinates, center, scale_factor=1):
    # Calculate the min and max bounds of the input coordinates
    x_min, y_min = coordinates.min(axis=0)
    x_max, y_max = coordinates.max(axis=0)

    # Determine the size of the bounding box
    x_range = x_max - x_min
    y_range = y_max - y_min

    # Scale the coordinates to fit within the range
    scaled_coords = (coordinates - [x_min, y_min]) / [x_range, y_range]  # Normalize to [0, 1]
    
    # Scale the normalized coordinates by the desired scale factor
    scaled_coords *= scale_factor  # Apply the scale factor
    
    # Offset the scaled coordinates to center them at the specified center point
    scaled_coords += [center[0] - scale_factor / 2, center[1] - scale_factor / 2]
    
    return scaled_coords

def plot_3d_coordinates(coordinates, center, scale_factor=1):
    # Convert coordinates into 3D (we'll assume z=0 for all points to start)
    z_vals = np.zeros(len(coordinates))
    
    # Scale and center the coordinates
    scaled_coords = scale_and_center_coordinates(coordinates, center, scale_factor)

    # Create a 3D plot
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the coordinates in 3D
    ax.scatter(scaled_coords[:, 0], scaled_coords[:, 1], z_vals, color='red', marker='o')

    # Set labels and title
    ax.set_title('3D Cartesian Plot')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Set equal aspect ratio (keeping the scaling consistent)
    ax.set_box_aspect([1, 1, 1])  # Equal scaling in all directions

    plt.show()

def main():
    coordinates = np.array([
        [1, 1],    # Point 1
        [4, 1],    # Point 2
        [2.5, 4]   # Point 3
    ])
    if len(coordinates) > 0:
        # Input the center and scale factor
        while True:
            center_input = input("Enter the center coordinate as [x, y] (e.g., [0.5, 0.5]): ")
            try:
                center = np.array(list(map(float, center_input.strip("[]").split(","))))
                if len(center) == 2:
                    break  # Valid input, exit the loop
                else:
                    print("Invalid center. Please enter a valid [x, y] coordinate.")
            except ValueError:
                print("Invalid input. Please enter the center as [x,y] (e.g., [0.5, 0.5]).")

        scale_factor_input = input("Enter the scale factor (default is 1): ")
        scale_factor = float(scale_factor_input) if scale_factor_input else 1
        
        plot_3d_coordinates(coordinates, center, scale_factor)
    else:
        print("No coordinates to plot.")

if __name__ == "__main__":
    main()
