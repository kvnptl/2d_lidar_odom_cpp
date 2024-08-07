# 2D Lidar Odometry

- Given 2D lidar scans, calculate the odometry using Iterative Closest Point (ICP) algorithm

## Algorithm

1. Initialization
   1. Build the KD-Tree for the target scan
   2. Initial transformation, rotation is set to identity, translation is set to zero
   3. Parameters for ICP, max_iterations = 100, tolerance = 1e-6
2. ICP loop
   1. For each iteration, find correspondences between the source scan and the target scan
   2. Calculate jacobians and errors for each correspondence
   3. Computer Hessian matrix H and error vector b
   4. Solve Hx = b for x
   5. Update the transformation
   6. Compute mean error for convergence check
3. Convergence check
   1. Check if the change in the mean error is less than the tolerance
   2. If not, go back to step 2 else break the loop
4. Return the final transformation
   1. Apply the final transformation to the source scan
   2. Visualize the source and target scans