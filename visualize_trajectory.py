import matplotlib.pyplot as plt
import numpy as np

# Load the trajectory data
data = np.loadtxt('trajectory_with_orientation.txt')
x = data[:, 0]
y = data[:, 1]
theta = data[:, 2]

# Plot the trajectory
plt.plot(x, y, 'r-')

# Plot the orientation arrows
for i in range(0, len(x), 10):  # Plot every 10th arrow for clarity
    plt.arrow(x[i], y[i], 0.1 * np.cos(theta[i]), 0.1 * np.sin(theta[i]), head_width=0.05, head_length=0.1, fc='blue', ec='blue')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D LiDAR Odometry Trajectory with Orientation')
plt.axis('equal')
plt.show()