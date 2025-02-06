import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from PythonRobotics.PathPlanning.ReedsSheppPath.reeds_shepp_path_planning import reeds_shepp_path_planning


#Define Goal Position:
start_pos = [0,-10,np.pi/2] #Vehicle looking straight ahead
goal_pos = [5,7.5,0]
r = 0.25 #Radius of tolerance around goal posistion
max_iter = 1000


env_bounds_y = 10.5  # Y-axis limit
env_bounds_x = 9  # X-axis limit
vehicle_dims = (1.7, 2.7)  # (Width, Length) of BMW X6
curvature = 1/7 

# Define obstacles as a list of rectangles
obstacles = []
for y in [-10, -5, 0, 5, 10]:
    for x in [-5, 5]:
        obstacles.append(Rectangle((x, y), 0.5, 0.5))

rrt_counter = 0


def in_object(pos):
    result = False
    pos_x = pos[0]
    pos_y = pos[1]
    for i in range(len(obstacles)):
        if obstacles[i].get_x() <= pos_x <= (obstacles[i].get_x() + obstacles[i].get_width()):
            if obstacles[i].get_y() <= pos_y <= (obstacles[i].get_y() + obstacles[i].get_height()):
                result = True
    
    return result


def closest_neighbour(point, nodes_list):
    closest_node = None
    closest_dist = 100000
    for i in nodes_list:
        dist = np.sqrt((point[0] - i[0])**2 + (point[1] - i[1])**2) + np.arctan(np.cos(point[2])*np.cos(i[2]) +  np.sin(point[2])*np.sin(i[2]))
        if dist < closest_dist:
            closest_node = i 
            closest_dist = dist
    
    return closest_node

def in_prox(point, goal_point, thresh = 0.25):
    dist = np.sqrt((point[0] - goal_point[0])**2 + (point[1] - goal_point[1])**2) + np.arctan(np.cos(point[2])*np.cos(goal_point[2]) +  np.sin(point[2])*np.sin(goal_point[2]))
    if dist<thresh:
        return True
    else:
        return False

nodes = [start_pos]
edges = []
paths = []
parent_map = {}

while rrt_counter < max_iter:
    p_new_x = np.random.uniform(-env_bounds_x, env_bounds_x)
    p_new_y = np.random.uniform(-env_bounds_y, env_bounds_y)
    p_new_theta = np.random.uniform(0, 2 * np.pi)
    p_new = [p_new_x, p_new_y, p_new_theta]
    print(p_new)

    if in_object(p_new) is False:  
        c_neighbour = closest_neighbour(p_new, nodes)
        if c_neighbour is not None:
            try:
                # Attempt Reeds-Shepp path planning
                path_x, path_y, path_yaw, mode, lengths = reeds_shepp_path_planning(
                    c_neighbour[0], c_neighbour[1], c_neighbour[2],
                    p_new[0], p_new[1], p_new[2],
                    curvature, step_size=0.1
                )

                # Check if any part of the path collides with an object
                path_in_patch = any(in_object([x, y]) for x, y in zip(path_x, path_y))

                if not path_in_patch:
                    print("Successful")
                    nodes.append(p_new)
                    paths.append([path_x, path_y, path_yaw])
                    edges.append([c_neighbour, p_new])
                    parent_map[tuple(p_new)] = tuple(c_neighbour)
                    if in_prox(p_new, goal_pos):
                        goal_node = tuple(p_new)
                        rrt_counter = max_iter  # Terminate early if goal is reached
            except Exception as e:
                print(f"Reeds-Shepp planning failed: {e}")
                pass  # Continue to the next iteration if planning fails

    rrt_counter += 1

successful_path = []
if goal_node in parent_map:
    current = goal_node
    while current in parent_map:
        successful_path.append(current)
        current = parent_map[current]
    successful_path.append(tuple(start_pos))  # Ensure we include the start
    successful_path.reverse()  # Start-to-goal order


final_path_x, final_path_y = [], []
for i in range(len(successful_path) - 1):
    start = successful_path[i]
    end = successful_path[i + 1]

    path_x, path_y, _, _, _ = reeds_shepp_path_planning(
        start[0], start[1], start[2], end[0], end[1], end[2],
        curvature, step_size=0.1
    )
    final_path_x.extend(path_x)
    final_path_y.extend(path_y)

# Plot environment
fig, ax = plt.subplots()
# ax.set_xlim(-env_bounds_x, env_bounds_x)
# ax.set_ylim(-env_bounds_y, env_bounds_y)
ax.set_title("Autonomous Parking Simulation")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(True)


for obs in obstacles:
    ax.add_patch(Rectangle(obs.xy, obs.get_width(), obs.get_height(), color="k"))

for node in nodes:
    ax.plot(node[0],node[1],"ro")

# for path in paths:
#     ax.plot(path[0], path[1], "k-")

ax.plot(final_path_x, final_path_y, "b-", linewidth=2, label="Optimal Path")


ax.plot(start_pos[0],start_pos[1],"go", markersize = 10)
ax.plot(goal_pos[0],goal_pos[1],"ro", markersize = 10)

# Add vehicle
ax.add_patch(Rectangle((0,0), 
                       vehicle_dims[0], vehicle_dims[1], fill=False, color="r"))

plt.show()