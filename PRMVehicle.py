import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from PythonRobotics.PathPlanning.ReedsSheppPath import reeds_shepp_path_planning

env_bounds_y = 10.5  # Y-axis limit
env_bounds_x = 9  # X-axis limit
vehicle_dims = (2, 4.9)  # (Width, Length) of BMW X6
disc_size = 10  # Grid discretization
dist_thresh = 2  # Path planning radius
curvature = 1/12.8  

# Define grid points
x_grid_points = np.linspace(-env_bounds_x, env_bounds_x, disc_size)[1:-1]
y_grid_points = np.linspace(-env_bounds_y, env_bounds_y, disc_size)[1:-1]
xv, yv = np.meshgrid(x_grid_points, y_grid_points)
angles = np.linspace(0, 360, 10, endpoint=False) * np.pi / 180  # in radians

# Define obstacles as a list of rectangles
obstacles = []
for y in [-10, -5, 0, 5, 10]:
    for x in [-5, 5]:
        obstacles.append(Rectangle((x, y), 0.5, 0.5))

def get_vehicle_corners(x, y, yaw):
    w, l = vehicle_dims
    half_w = w / 2
    half_l = l / 2

    # Corners relative to the center
    corners = [
        (-half_l, -half_w),
        (-half_l, half_w),
        (half_l, half_w),
        (half_l, -half_w)
    ]

    # Rotate each corner by yaw
    rotated_corners = []
    for dx, dy in corners:
        x_rot = dx * np.cos(yaw) - dy * np.sin(yaw)
        y_rot = dx * np.sin(yaw) + dy * np.cos(yaw)
        rotated_corners.append((x + x_rot, y + y_rot))

    return rotated_corners

def is_collision(x, y, yaw, obstacles):
    corners = get_vehicle_corners(x, y, yaw)
    for (cx, cy) in corners:
        for obs in obstacles:
            obs_x, obs_y = obs.xy
            obs_w = obs.get_width()
            obs_h = obs.get_height()
            if (obs_x <= cx <= obs_x + obs_w) and (obs_y <= cy <= obs_y + obs_h):
                return True
    return False

def get_neighbors(x_curr, y_curr, x_other, y_other):
    neighbors = []
    for start_angle in angles:
        for end_angle in angles:
            # Generate Reeds-Shepp path
            try:
                path_x, path_y, path_yaw, mode, lengths, total_length = reeds_shepp_path_planning(
                    x_curr, y_curr, start_angle,
                    x_other, y_other, end_angle,
                    curvature, step_size=0.1
                )
            except Exception as e:
                print(f"Error planning path: {e}")
                continue
            
            if total_length <= dist_thresh:
                collision = False
                for x, y, yaw in zip(path_x, path_y, path_yaw):
                    if is_collision(x, y, yaw, obstacles):
                        collision = True
                        break
                if not collision:
                    neighbors.append({
                        'start_angle': start_angle,
                        'end_angle': end_angle,
                        'path_x': path_x,
                        'path_y': path_y,
                        'total_length': total_length
                    })
    return neighbors

# Plot environment
fig, ax = plt.subplots()
ax.set_xlim(-env_bounds_x, env_bounds_x)
ax.set_ylim(-env_bounds_y, env_bounds_y)
ax.set_title("Autonomous Parking Simulation")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(True)

for obs in obstacles:
    ax.add_patch(Rectangle(obs.xy, obs.get_width(), obs.get_height(), color="k"))

ax.plot(xv, yv, "ro")

# Example usage: get neighbors for a point and plot paths
current_x, current_y = 0, -9  # Vehicle's initial position
for x_other in x_grid_points:
    for y_other in y_grid_points:
        # Check if other point is within Euclidean distance threshold (optional)
        if np.hypot(current_x - x_other, current_y - y_other) > dist_thresh:
            continue
        neighbors = get_neighbors(current_x, current_y, x_other, y_other)
        for neighbor in neighbors:
            ax.plot(neighbor['path_x'], neighbor['path_y'], 'b-', alpha=0.1)

# Add vehicle
ax.add_patch(Rectangle((current_x - vehicle_dims[0]/2, current_y - vehicle_dims[1]/2), 
                       vehicle_dims[0], vehicle_dims[1], alpha=1, fill=False, color="r"))

plt.show()