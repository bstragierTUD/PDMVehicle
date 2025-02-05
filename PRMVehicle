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

#Define grid points
x_grid_points = np.linspace(-env_bounds_x, env_bounds_x, disc_size)[1:-1]
y_grid_points = np.linspace(-env_bounds_y, env_bounds_y, disc_size)[1:-1]
xv, yv = np.meshgrid(x_grid_points, y_grid_points)
angles = np.linspace(0, 360, 10, endpoint=False) * np.pi / 180  # in radians

# Define obstacles as a list of rectangles
obstacles = []
for y in [-10, -5, 0, 5, 10]:
    for x in [-5, 5]:
        obstacles.append((Rectangle((x, y), 0.5, 0.5)))

def get_neighbors(x_curr, y_curr, x_other,y_other):
    neighbors = []
    
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

# Add vehicle
ax.add_patch(Rectangle((0, -9), vehicle_dims[0], vehicle_dims[1], alpha=1, fill=False, color="r"))

plt.show()