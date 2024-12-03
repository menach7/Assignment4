import numpy as np
import matplotlib.pyplot as plt

# Ideal square path coordinates
ideal_path = np.array([[0, 0], [0, 10], [5, 10], [5, 0]])

# Function to calculate the distance from a point to a line segment
def point_to_line_distance(px, py, ax, ay, bx, by):
    # Vector AB
    AB = np.array([bx - ax, by - ay])
    # Vector AP
    AP = np.array([px - ax, py - ay])
    # Project vector AP onto AB
    AB_mag = np.linalg.norm(AB)
    if AB_mag == 0:
        return np.linalg.norm(AP)  # A and B are the same point
    projection = np.dot(AP, AB) / AB_mag
    # Clamp the projection to the segment [A, B]
    projection = max(0, min(1, projection / AB_mag))
    closest_point = np.array([ax, ay]) + projection * AB
    return np.linalg.norm(np.array([px, py]) - closest_point)

# Function to read robot positions from a file
def read_robot_positions(filename):
    positions = []
    with open(filename, 'r') as file:
        for line in file:
            x, y, _ = map(float, line.split(','))
            positions.append([x, y])
    return np.array(positions)

# Read robot positions from the file
robot_positions = read_robot_positions('robot_position.txt')

# List to store the deviations from the ideal path
distances = []

# For each robot position, calculate the minimum distance to the ideal path
for position in robot_positions:
    min_distance = float('inf')
    for i in range(len(ideal_path)):
        # Check the distance to each edge of the square
        j = (i + 1) % len(ideal_path)  # Next point to form a segment
        distance = point_to_line_distance(position[0], position[1], ideal_path[i][0], ideal_path[i][1], ideal_path[j][0], ideal_path[j][1])
        min_distance = min(min_distance, distance)
    distances.append(min_distance)

# Convert distances to a numpy array for analysis
distances = np.array(distances)

# Plotting the results
plt.figure(figsize=(10, 8))

# Plot the ideal path (square)
plt.plot([ideal_path[0][0], ideal_path[1][0]], [ideal_path[0][1], ideal_path[1][1]], 'r-', label='Ideal Path')
plt.plot([ideal_path[1][0], ideal_path[2][0]], [ideal_path[1][1], ideal_path[2][1]], 'r-')
plt.plot([ideal_path[2][0], ideal_path[3][0]], [ideal_path[2][1], ideal_path[3][1]], 'r-')
plt.plot([ideal_path[3][0], ideal_path[0][0]], [ideal_path[3][1], ideal_path[0][1]], 'r-')

# Plot the robot's path
plt.plot(robot_positions[:, 0], robot_positions[:, 1], 'b-', label='Robot Path')

# Add labels and title
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Comparison of Robot Path to Ideal Square Path')
plt.legend()

# Show the plot
plt.grid(True)
plt.show()

# Statistical analysis of the accuracy
avg_distance = np.mean(distances)
max_distance = np.max(distances)
min_distance = np.min(distances)

print(f"Average distance from ideal path: {avg_distance:.4f}")
print(f"Maximum distance from ideal path: {max_distance:.4f}")
print(f"Minimum distance from ideal path: {min_distance:.4f}")

# Optionally: Plot a histogram of the deviations
plt.figure(figsize=(8, 6))
plt.hist(distances, bins=20, color='c', edgecolor='black')
plt.title('Histogram of Deviations from Ideal Path')
plt.xlabel('Deviation (Distance)')
plt.ylabel('Frequency')
plt.grid(True)
plt.show()

