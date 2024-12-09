import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

"""
Last Updated: December 4, 2024 

@author: Temple 
"""



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
            dU -= 1.5*compute_attpotgrad(
                np.array(goal_pos_world[:2]),
                np.array(robot_pos_world[:2]),
                eps1=620,
                eps2=130,
                max_thresh=0.2,
            )


            # To Handle Robot-Robot Repulsion if necessary(Now relatively safe)
            # Add robot-robot repulsion to the gradient
            for other_robot_name, other_robot_pos in shared_robot_positions.items():
                if other_robot_name != robot_name:
                    dU -= compute_robot_repulsion(
                        np.array(robot_pos_world[:2]),
                        other_robot_pos
                    )


            # To try when we detect path-crossing conflicts(Now relatively safe)
            # Resolve conflicts based on proximity and priority
            dU = resolve_conflicts(robot_name, dU, priority_dict, shared_robot_positions, conflict_dist=1.0)

            # Smooth the gradient for better transitions
            dU = smooth_gradient(dU, robot_name)

            # Set the wheel velocities for the respective robot
            set_wheel_velocity(dU, sim, f'set_F@/Pure_Robot[{robot_name}]')