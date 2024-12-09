import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import ecse275utils as util

"""
Last Updated: November 24, 2024 

@author: Temple 
"""
# Work on robot path optimization
# Other secondary goals should be given priority next

# %%
# Still need some work on smooth transition check version 2
# Define smoothing parameters and helper functions


# Smoothing factor for exponential smoothing
alpha = 0.8

# Dictionary to store previous gradients(For the smoothing function)
previous_dU = {"0": np.zeros(2), "1": np.zeros(2), "2": np.zeros(2)}

# Initialize shared dictionary for shared position to be used for ball recognition
shared_robot_positions = {"0": np.zeros(2), "1": np.zeros(2), "2": np.zeros(2)}

# Priorities for robots when there is path intersection which robot goes first
priority_dict = {"0": 1, "1": 2, "2": 3}

# Define some parameters to be used to smoothly stop the robot when close to the goal(Prevent undershoot & Overshoot)
decel_time = 1.0  # Deceleration period in seconds
time_step = 0.15  # Time step for gradual reduction

def smooth_stop(robot_name, sim, scriptfuncname):
    """
    Smoothly stops the robot by gradually reducing wheel velocities and gradients.
    Parameters:
    -----------
    robot_name : str
        The name of the robot.
    sim : object
        Simulation object.
    scriptfuncname : str
        The name of the script function for setting wheel velocities.
    """
    global previous_dU

    # Get the current gradient and velocity
    current_dU = previous_dU[robot_name]
    print(robot_name)

    # Gradually reduce the gradient and velocities to zero
    steps = int(decel_time / time_step)
    for step in range(steps, 0, -1):
        factor = step / steps  # Linearly scale down
        smoothed_dU = factor * current_dU
        set_wheel_velocity(smoothed_dU, sim, scriptfuncname)  # Apply scaled-down velocity
        time.sleep(time_step)  # Wait for the time step

    # Final step: ensure velocities and gradients are exactly zero
    set_wheel_velocity(np.array([0.0, 0.0]), sim, scriptfuncname)
    previous_dU[robot_name] = np.array([0.0, 0.0])  # Reset gradient



def smooth_gradient(current_dU, robot_name):
    """
    Applies exponential smoothing to the gradient for smoother transitions.
    """
    global previous_dU
    smoothed_dU = alpha * previous_dU[robot_name] + (1 - alpha) * current_dU
    previous_dU[robot_name] = smoothed_dU  # Update the previous gradient
    return smoothed_dU


def get_range_data(sim, scriptfuncname, transform_pose=None):
    """
    Retrieves range data from the robot's sensors and transforms it into the world frame.
    """
    output = sim.callScriptFunction(scriptfuncname, sim.scripttype_childscript)
    try:
        output_robotframe = np.array(output).reshape((-1, 3))
    except:
        output_robotframe = np.zeros_like(output_robotframe)

    if transform_pose is not None:
        robot_rotmat = R.from_quat(transform_pose[3:])
        output_robotframe = robot_rotmat.apply(output_robotframe) + transform_pose[:3]

    return output_robotframe


def set_wheel_velocity(dU, sim, scriptfuncname):
    """
    Sets wheel velocities based on the computed control gradient.
    """
    sim.callScriptFunction(scriptfuncname, sim.scripttype_childscript, [dU[0], dU[1]])


def compute_reppotgrad(point, robot_pos, eps=3, min_thresh=1, max_thresh=15):
    """
    Computes the gradient of the repulsive potential between a point (obstacle) and the robot.
    """
    d = np.linalg.norm(point - robot_pos)
    if min_thresh < d < max_thresh:
        return (-1 * eps * (np.array(robot_pos) - np.array(point))) / (d * d * d)
    return np.array([0.0, 0.0])


def compute_attpotgrad(point, robot_pos, eps1=5, eps2=5, max_thresh=5):
    """
    Computes the gradient of the attractive potential between a point (goal) and the robot.
    """
    d = np.linalg.norm(point - robot_pos)
    if d < max_thresh:
        return eps1 * (np.array(robot_pos) - np.array(point))
    return (eps2 * max_thresh * (np.array(robot_pos) - np.array(point))) / d


def compute_robot_repulsion(robot_pos, other_robot_pos, safe_dist=0.5, max_dist=2, repulsion_strength=3):
    """
    Computes the repulsive potential gradient to avoid collisions with other robots.
    """
    d = np.linalg.norm(robot_pos - other_robot_pos)
    if safe_dist < d < max_dist:
        return -repulsion_strength * (robot_pos - other_robot_pos) / (d ** 3)
    return np.zeros(2)


def resolve_conflicts(robot_name, dU, priority_dict, shared_positions, conflict_dist=1.0):
    """
    Adjusts the gradient based on dynamic priority, but only when robots are in close proximity.

    Parameters:
    -----------
    robot_name : str
        The identifier of the current robot (e.g., "0", "1", "2").
    dU : numpy array
        The current gradient of the robot.
    priority_dict : dict
        A dictionary mapping robot names to their priority values.
    shared_positions : dict
        A dictionary of robot positions.
    conflict_dist : float
        The distance threshold to consider robots in a conflict zone.

    Returns:
    --------
    adjusted_dU : numpy array
        The adjusted gradient vector based on proximity-based priority resolution.
    """
    current_position = shared_positions[robot_name]
    for other_robot_name, other_position in shared_positions.items():
        if other_robot_name != robot_name:
            distance = np.linalg.norm(current_position - other_position)
            # Apply priority-based adjustment only if within conflict distance
            if distance < conflict_dist and priority_dict[other_robot_name] > priority_dict[robot_name]:
                dU *= 0.5  # Reduce movement to yield to higher-priority robot
    return dU

def rep_hyper_vec(distances, eta=1, max_dist=2, min_dist=1):
    """
    Computes the repulsive potential for multiple points using a hyperbolic function.
    """
    U = np.zeros_like(distances)
    mask = (distances >= max_dist)
    U[mask] = 0
    mask = (min_dist <= distances) & (distances < max_dist)
    distance = distances[mask]  # Points within min_dist and max_dist
    U[mask] = eta * (1 / distance - 1 / max_dist)
    mask = (distances < min_dist)  # Points closer than min_dist
    U[mask] = eta * (1 / min_dist - 1 / max_dist)
    return U
    #return U * 0.5

def att_quadcone_vec(dist, eta1=1, eta2=10, thresh=2):
    """
    Computes the attractive potential for multiple points using a hybrid quadratic-conic function.
    """
    U = np.zeros_like(dist)
    h, w = U.shape
    mask = (dist <= thresh)
    U = U.reshape((-1))
    mask = mask.reshape((-1))
    dist = dist.reshape((-1))
    distance = dist[mask]  # Below the threshold
    U[mask] = 0.5 * eta1 * np.power(distance, 2)
    mask = (dist > thresh)  # Above the threshold
    distance = dist[mask]
    U[mask] = thresh * eta2 * (distance) - eta1 * (thresh**2) * 0.5
    #U[mask] = thresh * eta2 * (distance) * 0.8  # Reduce distant influence (NEW CHANGE)
    return U.reshape(h, w)


# Main control logic
run_reactive_control = True
stop_condition = 0.5  # meters
goal_transition_delay = 5  # seconds


if __name__ == '__main__':
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')

    # Get handles for robots
    robots = [
        sim.getObjectHandle("/Pure_Robot[0]/Dummy"),
        sim.getObjectHandle("/Pure_Robot[1]/Dummy"),
        sim.getObjectHandle("/Pure_Robot[2]/Dummy")
    ]

    # Define goal queues for each robot
    goal_queues = {
        "0": [sim.getObjectHandle("/goal_point[0]"), sim.getObjectHandle("/goal_point[3]")],
        "1": [sim.getObjectHandle("/goal_point[1]"), sim.getObjectHandle("/goal_point[4]")],
        "2": [sim.getObjectHandle("/goal_point[2]"), sim.getObjectHandle("/goal_point[5]")]
    }

    # Track current goal index for each robot
    current_goal_index = {"0": 0, "1": 0, "2": 0}

    # Track completion status for each robot
    completed = {"0": False, "1": False, "2": False}
    
    
    # Tuning parameters
    repulsive_eta = 2
    repulsive_max_dist = 4
    repulsive_min_dist = 1

    attractive_eta1 = 0.1
    attractive_eta2 = 0.2
    attractive_thresh = 5
    
    # ---------------------------------------------------------------------------------------------------------------------------

    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')

    # Initialize lists to hold robot and goal positions
    robots = []
    robot_positions = []
    goals = []
    goal_positions = []
    goal_grids = []

    # Retrieve all robot and goal handles and positions
    for i in range(3):  # For Pure_Robot[0] to Pure_Robot[2]
        robot_handle = sim.getObjectHandle(f"/Pure_Robot[{i}]")
        robots.append(robot_handle)
        robot_pos = sim.getObjectPosition(robot_handle, sim.handle_world)
        robot_positions.append(robot_pos)

    for i in range(6):  # For goal_point[0] to goal_point[5]
        goal_handle = sim.getObjectHandle(f"/goal_point[{i}]")
        goals.append(goal_handle)
        goal_pos = sim.getObjectPosition(goal_handle, sim.handle_world)
        goal_positions.append(goal_pos)

    # Initialize the Grid Map
    worldmap = util.gridmap(sim, 15.0, goal_positions[0], robot_positions[0], inflate_iter=1)

    # Convert goal positions to grid coordinates
    for goal_pos in goal_positions:
        goal_grids.append(worldmap.get_grid_coords(goal_pos))

    # Visualize the map
    worldmap.plot(normalized=True)
    for goal_grid in goal_grids:
        worldmap.plot_coord(goal_grid)

    # Compute a grid of distances for obstacles
    grid_height, grid_width = worldmap.gridmap.shape
    eval_points = np.indices((grid_height, grid_width), dtype=float).transpose(1, 2, 0)  # Create the evaluation grid
    obs_idx = np.argwhere(worldmap.norm_map == 1)  # Get indices of obstacles
    distances_rep = np.linalg.norm(np.expand_dims(eval_points, axis=2) - np.expand_dims(obs_idx, axis=0), axis=3)

    # Compute the repulsive field (static for all robot-goal pairs)
    rep_potentials = np.apply_along_axis(
        rep_hyper_vec, axis=-1, arr=distances_rep, eta=repulsive_eta, max_dist=repulsive_max_dist, min_dist=repulsive_min_dist
    )
    repulsive_field = np.sum(rep_potentials, axis=-1)

    # Define robot-goal groups (predefined)
    robot_goal_groups = {
        0: [0, 3],  # Robot 0, Goals 0 and 3
        1: [1, 4],  # Robot 1, Goals 1 and 4
        2: [2, 5],  # Robot 2, Goals 2 and 5
    }

    # Predefined groups: Per-robot visualization
    for robot_idx, goal_indices in robot_goal_groups.items():
        print(f"Processing Robot {robot_idx} with Goals {goal_indices}")

        # Get robot position
        robot_grid = worldmap.get_grid_coords(robot_positions[robot_idx])

        # Initialize combined attractive field for the selected goals
        combined_attractive_field = np.zeros((grid_height, grid_width))
    
        # Compute the attractive potentials for the two specified goals
        for goal_idx in goal_indices:
            distances_att = np.linalg.norm(
                np.expand_dims(eval_points, axis=2) - np.expand_dims(goal_grids[goal_idx], axis=0), axis=-1
            )
            distances_att = np.reshape(distances_att, (grid_height, grid_width))

            attractive_field = att_quadcone_vec(
                distances_att, eta1=attractive_eta1, eta2=attractive_eta2, thresh=attractive_thresh
            )
            combined_attractive_field += attractive_field

        # Combine repulsive and attractive fields for this robot
        robot_potential_field = repulsive_field + combined_attractive_field

        # Visualize the individual potential field and heatmap for this robot
        #util.visualize_potential_field_3D(robot_potential_field)
        
        # Compute the gradient and path for the robot to its first goal
        world_ugrad = util.compute_discrete_gradient(robot_potential_field)
        util.visualize_gradient(
            worldmap.norm_map, world_ugrad.transpose(1, 2, 0), np.flip(goal_grids[goal_indices[0]])
        )
        path = util.discrete_grad_descent(robot_grid, goal_grids[goal_indices[0]], world_ugrad, max_iter=5000)
        worldmap.plot(normalized=True)
        util.plot_gradient_descent(plt.gca(), path)
        util.visualize_potential_field_2D(robot_potential_field)
    
    # Compute and visualize one combined global potential field for all robots and goals

    # Compute the global attractive field for all goals
    global_attractive_field = np.zeros((grid_height, grid_width))
    for goal_idx in range(len(goal_grids)):
        distances_att = np.linalg.norm(
            np.expand_dims(eval_points, axis=2) - np.expand_dims(goal_grids[goal_idx], axis=0), axis=-1
        )
        distances_att = np.reshape(distances_att, (grid_height, grid_width))

        attractive_field = att_quadcone_vec(
            distances_att, eta1=attractive_eta1, eta2=attractive_eta2, thresh=attractive_thresh
        )
        global_attractive_field += attractive_field

    # Combine the global attractive field with the repulsive field
    global_potential_field = repulsive_field + global_attractive_field
    
    # Normalize and scale for better visualization
    global_potential_field = global_potential_field - np.min(global_potential_field)  # Offset to start at 0
    global_potential_field = global_potential_field / np.max(global_potential_field)  # Normalize to [0, 1]
    global_potential_field = global_potential_field * 10  # Scale to emphasize terrain features (NEW CHANGE)
   
    # Visualize the combined global potential field as a single 3D surface plot
    print("Visualizing Combined Global Potential Field")
    util.visualize_potential_field_3D(global_potential_field)
    # ---------------------------------------------------------------------------------------------------------------------------

    # Main control loop for all robots
    while run_reactive_control:
        # Check if all robots have completed their goal queues
        if all(completed.values()):
            print("All robots have completed their goals. Exiting loop.")
            break  # Exit the loop when all robots are done

        # Loop through each robot
        for robot, robot_name in zip(robots, ["0", "1", "2"]):
            # Skip the robot if it has completed its goal queue
            if completed[robot_name]:
                continue

            # Get current goal handle
            goal_index = current_goal_index[robot_name]
            current_goal = goal_queues[robot_name][goal_index]

            # Get robot and goal positions
            robot_pose = sim.getObjectPose(robot, sim.handle_world)
            robot_pos_world = robot_pose[:3]
            goal_pos_world = sim.getObjectPosition(current_goal, sim.handle_world)

            # Update shared robot positions for communication
            shared_robot_positions[robot_name] = np.array(robot_pos_world[:2])

            # Initialize the gradient descent vector
            dU = np.array([0.0, 0.0])

            # Check if the robot is close to its current goal
            if np.linalg.norm(np.array(robot_pos_world[:2]) - np.array(goal_pos_world[:2])) < stop_condition:
                print(f"Robot {robot_name} reached goal {goal_index}")

                # Smoothly stop the robot
                smooth_stop(robot_name, sim, f'set_F@/Pure_Robot[{robot_name}]')

                # Wait for the transition delay
                time.sleep(goal_transition_delay)

                # Check if the robot has reached its second (final) goal
                if goal_index == len(goal_queues[robot_name]) - 1:
                    completed[robot_name] = True  # Mark robot as completed
                    print(f"Robot {robot_name} completed all goals.")
                else:
                    # Update to the next goal in the queue
                    current_goal_index[robot_name] += 1

                continue  # Skip the rest of the loop to handle goal switching

            # Get range data for obstacle avoidance
            rangedata_worldframe = get_range_data(
                sim, f'getMeasuredData@/Pure_Robot[{robot_name}]/fastHokuyo', transform_pose=robot_pose
            )

            # Compute repulsive gradient for obstacles
            for i in range(rangedata_worldframe.shape[0]):
                dU -= 1.5*compute_reppotgrad(
                    rangedata_worldframe[i, 0:2],
                    np.array(robot_pos_world[:2]),
                    eps=0.02,
                    min_thresh=0.03,
                    max_thresh=0.5,
                )

            # Compute attractive gradient for the current goal
            dU -= compute_attpotgrad(
                np.array(goal_pos_world[:2]),
                np.array(robot_pos_world[:2]),
                eps1=620,
                eps2=130,
                max_thresh=0.2,
            )

            """
            # To Handle Robot-Robot Repulsion if necessary(Now relatively safe)
            # Add robot-robot repulsion to the gradient
            for other_robot_name, other_robot_pos in shared_robot_positions.items():
                if other_robot_name != robot_name:
                    dU -= compute_robot_repulsion(
                        np.array(robot_pos_world[:2]),
                        other_robot_pos
                    )
            """

            # To try when we detect path-crossing conflicts(Now relatively safe)
            # Resolve conflicts based on proximity and priority
            #dU = resolve_conflicts(robot_name, dU, priority_dict, shared_robot_positions, conflict_dist=1.0)

            # Smooth the gradient for better transitions
            dU = smooth_gradient(dU, robot_name)

            # Set the wheel velocities for the respective robot
            set_wheel_velocity(dU, sim, f'set_F@/Pure_Robot[{robot_name}]')