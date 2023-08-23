"""
Unicycle Adaptive Headway Motion Control and Distance Gradient Motion Control
"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# For the union function to compute the truncated ice cream cone
from shapely.geometry import Polygon

from geometry_tools import circle_polygon, convexhull_polygon


####################################################################################
############################ UNICYCLE DYNAMICS #####################################
####################################################################################

def unicycle_discrete_update(position, orientation, linear_velocity, angular_velocity, dt):
  """
  Computes unicycle discrete state update assuming contant velocity 
  """
  position[0] = position[0] + linear_velocity * np.cos(orientation) * dt
  position[1] = position[1] + linear_velocity * np.sin(orientation) * dt
  orientation = orientation + angular_velocity * dt
  return position, orientation

def unicycle_discrete_update_ode(position, orientation, linear_velocity, angular_velocity, dt, **kwargs):
  
  initial_pose = np.array([position[0], position[1], orientation])

  unicycle_dynamics = lambda t, pose: np.array([linear_velocity*np.cos(pose[2]), linear_velocity*np.sin(pose[2]), angular_velocity])

  sol = solve_ivp(unicycle_dynamics, [0, dt], initial_pose, **kwargs)

  position = [sol.y[0][-1], sol.y[1][-1]]
  orientation = sol.y[2][-1]

  return position, orientation


####################################################################################
############################ UNICYCLE CONTROL FUNCTIONS ############################
####################################################################################

def adaptive_headway_ctrl(position, orientation, goal, epsgain = 0.5, refgain = 1, tol=1e-3, motion_direction = 'Bidirectional'):
    """
    Computes motion control inputs for the adaptive headway control of a robot towards a given goal location.

    Function:
    linvel, angvel = adaptive_headway_ctrl(position, orientation, goal, epsgain, refgain, tol, motion_direction)

    Input:
    position: Current position of the robot [float, float]
    orientation: Current orientation (angle) of the robot [float]
    goal: Goal position [float, float]
    epsgain: Epsilon gain parameter for control [float], (default: 0.5)
    refgain: Reference gain parameter for control [float], (default: 1)
    tol: Motion control tolerance [float], (default: 1e-3)
    motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')

    Output:
    linvel: Linear velocity for motion control (unit/s) [float]
    angvel: Angular velocity for motion control (rad/s) [float]

    Usage:
    position = [0.0, 0.0]
    orientation = 0.0
    goal = [1.0, 0.0]
    epsgain = 0.5
    refgain = 1
    tol = 1e-3
    motion_direction = 'Bidirectional'
    linvel, angvel = adaptive_headway_ctrl(position, orientation, goal, epsgain, refgain, tol)
    """


    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position
    tol = np.float64(tol) # Tolerance
    motion_direction = motion_direction.lower()

    if not (motion_direction == 'forward' or motion_direction == 'backward' or motion_direction == 'bidirectional'):
        raise ValueError("Motion Direction must be 'Forward', 'Backward', or 'Bidirectional'")

    # Check motion control tolerance
    if (np.linalg.norm(position-goal) < tol):
        return 0.0, 0.0
    
    if motion_direction == 'backward':
        orientation = orientation + np.pi

    vdir = np.array([np.cos(orientation), np.sin(orientation)]) # Current Direction
    ndir = np.array([-np.sin(orientation), np.cos(orientation)]) # Normal Direction

    vproj = np.dot(goal-position, vdir) # Projection onto velocity direction
    nproj = np.dot(goal-position, ndir) # Projection onto velocity normal

    linvel = refgain * np.linalg.norm(goal-position) * (vproj/np.linalg.norm(goal-position) - epsgain) / (1 - epsgain*vproj/np.linalg.norm(goal-position))
    angvel = (refgain / epsgain) * nproj / np.linalg.norm(goal - position)

    # Turn in place with a constant and continuous angular velocity until the robot is aligned with the goal
    if motion_direction == 'forward' or motion_direction == 'backward':
        if linvel < 0:
            angvel = (refgain / epsgain) * np.sqrt(1 - epsgain**2)
            if (np.arctan2(nproj, vproj) <= 0):
                angvel = -1 * angvel
        linvel = np.maximum(linvel, 0)

    if motion_direction == 'backward':
        linvel = -1 * linvel

    return linvel, angvel

################################################################################################################
############################ ADAPTIVE HEADWAY CONTROL DYNAMICS AND MOTION PREDICTION ###########################
################################################################################################################

def ahc_vector_field(position, orientation, goal, epsgain, refgain, tol, motion_direction = 'Bidirectional'):
    """
    Unicycle dynamics for the kinematic unicycle robot model controlled via adaptive headway control.
    """

    position = np.array(position)
    goal = np.array(goal)
    
    linear_velocity, angular_velocity = adaptive_headway_ctrl(position, orientation, goal, epsgain, refgain, tol, motion_direction)
    dx = linear_velocity*np.cos(orientation)
    dy = linear_velocity*np.sin(orientation)
    dtheta = angular_velocity
    dX = np.array([dx, dy ,dtheta])

    return dX


def ahc_trajectory(position, orientation, goal, tspan, epsgain, refgain, tol, motion_direction = 'Bidirectional', **kwargs):

    """
    Computes the trajectory of the kinematic unicycle robot model controlled via adaptive headway control.


    Function:
    t, y = ahc_trajectory(position, orientation, goal, tspan, epsgain, **kwargs)

    Input:
    position: Current position of the robot [float, float]
    orientation: Current orientation (angle) of the robot [float]
    goal: Goal position [float, float]
    tspan: Time span for trajectory [float, float]
    epsgain: Control gain [float]
    refgain: Reference gain [float]
    tol: Tolerance for motion control [float]
    motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')
    **kwargs: Keyword arguments for solve_ivp

    Output:
    t: Time vector [array]
    y: State vector [array]
    """
    unicycle_dynamics = lambda t, X: ahc_vector_field(X[0:2], X[2], goal, epsgain, refgain, tol, motion_direction)

    position = np.array(position)
    initial_pose = np.append(position, orientation)

    sol = solve_ivp(unicycle_dynamics, tspan, initial_pose, max_step=1e-2, **kwargs)

    # position = [sol.y[0][-1], sol.y[1][-1]]
    # orientation = sol.y[2][-1]
    return sol.t, sol.y.T


def ahc_discrete_update(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction = 'Bidirectional', **kwargs):

    t, y = ahc_trajectory(position, orientation, goal, [0, duration], epsgain, refgain, tol, motion_direction, **kwargs)

    position = [y[-1][0], y[-1][1]]
    orientation = y[-1][2]

    return position, orientation


def ahc_discrete_update_map(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction = 'Bidirectional', **kwargs):
    ahc_discrete_update(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction, **kwargs)


def ahc_circle_motion_prediction(position, orientation, goal, motion_direction = 'Bidirectional', res = 60, epsgain = 0.5):
    """
    Computes the circular motion prediction for a robot model.

    Args:
        position: Current position of the robot [float, float]
        orientation: Current orientation (angle) of the robot [float]
        goal: Goal position [float, float]
        epsgain: Control gain [float]
        motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')
        res: Resolution for circle approximation [int]

    Returns:
        P: Motion polygon vertices [array]
    """

    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position
    position = position.flatten()
    goal = goal.flatten()

    # subtract goal from the position
    position = position - goal

    # Compute the headway distance and position
    headwaydist = epsgain * np.linalg.norm(position)
    headway_position = position + headwaydist * np.array([np.cos(orientation), np.sin(orientation)])

    # if the distance between the headway_position and the goal is zero, then the motionpolygon_radius is zero
    if headwaydist < 1e-3:
        motionpolygon_radius = 0
    else:
    
        # Compute the projected distance and extended distance
        projected_distance = np.dot(position, headway_position) / np.linalg.norm(headway_position)
        extended_distance = projected_distance / np.sqrt(1 - epsgain ** 2)

        # Compute the motion polygon radius based on the alignment condition
        motionpolygon_radius = extended_distance
        if np.dot(np.array([np.cos(orientation), np.sin(orientation)]), -position / np.linalg.norm(position)) >= epsgain:
            motionpolygon_radius = np.linalg.norm(position)

    # if the robot is moving forward or backward, the distance to the goal defines the motionpolygon_radius
    if motion_direction == 'forward' or motion_direction == 'backward':
        motionpolygon_radius = np.linalg.norm(position)

    # Compute the convex hull of the circle approximation
    P = circle_polygon(motionpolygon_radius, res)

    # Add each vertex to the polygon the goal
    for i in range(len(P)):
        P[i] = P[i] + goal

    return P


def ahc_triangle_motion_prediction(position, orientation, goal, motion_direction = 'Bidirectional', res = 60, epsgain = 0.5):
    """
    Computes the triangular motion prediction for a 2D Unicycle kinematic robot controlled via adaptive headway control.

    Args:        
        position: Current position of the robot [float, float]
        orientation: Current orientation (angle) of the robot [float]
        goal: Goal position [float, float]
        epsgain: Scalar control gain for adaptive headway distance [float]
        motion_direction: Direction of motion control, 'Bidirectional', 'Forward', 'Backward' [string], (default: 'Bidirectional')

    Returns:
        P: Motion polygon vertices [array]
    """

    if motion_direction == 'backward':
        orientation = orientation + np.pi

    position = np.array(position) # Current Position
    goal = np.array(goal) # Goal Position
    position = position.flatten()
    goal = goal.flatten()

    # subtract goal from the position
    position = position - goal

    if np.linalg.norm(position) == 0:
        P = np.array([goal, goal, goal])
        return P
    
    vdir = np.array([np.cos(orientation), np.sin(orientation)]) # Current Direction
    ndir = np.array([-np.sin(orientation), np.cos(orientation)]) # Normal Direction

    vproj = np.dot(-position, vdir) # Projection onto velocity direction
    nproj = np.dot(-position, ndir) # Projection onto velocity normal

    # Compute the headway distance and position
    headwaydist = epsgain * np.linalg.norm(position)
    headway_position = position + headwaydist * np.array([np.cos(orientation), np.sin(orientation)])

    if np.linalg.norm(position) != 0:
        t_e = headway_position / np.linalg.norm(headway_position)
    else:
        t_e = np.array([0, 0])

    if np.dot(position, np.array([-np.sin(orientation), np.cos(orientation)]) ) > 0:
        n_e = np.dot(np.array([[np.cos(np.pi/2), -np.sin(np.pi/2)], [np.sin(np.pi/2), np.cos(np.pi/2)]]), t_e)
    else:
        n_e = np.dot(np.array([[np.cos(-np.pi/2), -np.sin(-np.pi/2)], [np.sin(-np.pi/2), np.cos(-np.pi/2)]]), t_e)

    projected_pos = np.dot(t_e, np.dot(t_e, position))
    projected_distance = np.linalg.norm(projected_pos)
    extended_pos = projected_pos + (epsgain / np.sqrt(1 - epsgain**2)) * projected_distance * n_e
    extended_reflected = 2 * projected_pos - extended_pos

    X = np.array([[0, 0], extended_pos, extended_reflected])

    # if the robot is well aligned, then the motion polygon is reduced
    if np.dot(np.array([np.cos(orientation), np.sin(orientation)]), (-position / np.linalg.norm(position))) >= epsgain:
        x3 = headway_position + (1 - np.dot(np.array([np.cos(orientation), np.sin(orientation)]), (-position / np.linalg.norm(position)))) / (1 - epsgain) * headwaydist * np.array([np.cos(orientation), np.sin(orientation)])
        X = np.array([position, x3, [0, 0]])

    # if the robot is moving forward or backward, the motion polygon is reduced
    if motion_direction == 'forward' or motion_direction == 'backward':
        # if the robot is already well aligned
        if np.dot(np.array([np.cos(orientation), np.sin(orientation)]), (-position / np.linalg.norm(position))) >= epsgain:
            X = np.array([position, headway_position, [0, 0]])
        # if the robot should turn in place first
        else:
            rot_dir = -1
            if (np.arctan2(nproj, vproj) <= 0):
                rot_dir = 1
            # rotate the position vector by arccos(epsgain)
            motion_headway = position + epsgain * np.dot(np.array([[epsgain, -rot_dir*np.sqrt(1 - epsgain**2)], [rot_dir*np.sqrt(1 - epsgain**2), epsgain]]), -position) 
            

            beta = np.arccos(epsgain) # positive valued angle between the motion headway to goal vector
            phi = np.arctan2(nproj, vproj) # angle from the robot orientation to the goal vector

            if np.abs(phi) >= beta:
                reflection_scale = (np.abs(phi) - beta)/(np.pi - beta)
                reflection_rot_dir = -1 * rot_dir
                reduced_reflected_motion_headway2 = position + reflection_scale * epsgain * np.dot(np.array([[epsgain, -reflection_rot_dir*np.sqrt(1 - epsgain**2)], [reflection_rot_dir*np.sqrt(1 - epsgain**2), epsgain]]), -position) 
                X = np.array([position, [0, 0], reduced_reflected_motion_headway2, motion_headway])
            else:
                X = np.array([position, [0, 0], motion_headway])


    P = convexhull_polygon(X)

    # add each vertex to the polygon the goal
    for i in range(len(P)):
        P[i] = P[i] + goal

    return P