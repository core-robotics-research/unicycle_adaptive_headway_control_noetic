"""
First-Order Time Governor Python Module
Author: Aykut Isleyen, a.isleyen@tue.nl
Created: July 2023
"""

import numpy as np
from scipy.integrate import solve_ivp
from path_tools import path_length, path_value

def governor_vector_field(k, safety_gain, convergence_rate, safety_level, path):
    # k: path parameter in terms of path length

    k = np.array(k)
    L, L2 = path_length(path)

    if k < 0 or k > L:
        print('The time governor path value is out of bounds')
        k = np.maximum(np.minimum(k, L), 0)
        dk = 0
        dk = np.array(dk)
        return dk

    dkSafe = safety_gain * safety_level  # Safe Parameter Rate
    dkConv = convergence_rate * (L - k)  # Converging Parameter Rate

    dk = np.minimum(dkConv, dkSafe)
    dk = np.maximum(dk, 0)

    return dk


def robot_governor_vector_field(X, safety_gain, convergence_rate, robot_radius, OccGrid2D, path, **kwargs):
        
    robot_position = np.array(X[0:2])
    robot_orientation = X[2]
    governor_path_parameter = np.array(X[3])
    governor_position = path_value(path, governor_path_parameter).flatten()

    path = np.array(path)

    # make all kwarg keys lower case
    kwargs = {k.lower(): v for k, v in kwargs.items()}
    
    # set low level control vector field
    control_fnc = kwargs['control']
    linvel, angvel = control_fnc(robot_position, robot_orientation, governor_position,**kwargs['control_args'])
    drobot = np.array([linvel*np.cos(robot_orientation), linvel*np.sin(robot_orientation), angvel])
    
    # get motion prediction function
    motion_prediction_fnc = kwargs['motion_prediction']
    motion_polygon = motion_prediction_fnc(robot_position, robot_orientation, governor_position,  **kwargs['motion_prediction_args'])

    # get safety level
    frame = 'world'
    distance, XPolygon, XMap = OccGrid2D.distance_to_collision_polygon(motion_polygon, frame)
    safety_level = distance - robot_radius

    # set governor vector field
    dgov = governor_vector_field(governor_path_parameter, safety_gain, convergence_rate, safety_level, path)

    # concatenate the two vector fields
    dX = np.concatenate((drobot.flatten(), dgov.flatten()), axis=0)

    return dX

def robot_governor_trajectory(robot_position, robot_orientation, governor_path_parameter, safety_gain, convergence_rate, robot_radius, OccGrid2D, path, duration, **kwargs):
        
        robot_position = np.array(robot_position)
        robot_orientation = np.array(robot_orientation)
        governor_path_parameter = np.array(governor_path_parameter)

        path = np.array(path)

        robot_position = robot_position.flatten()
        robot_orientation = robot_orientation.flatten()
        governor_path_parameter = governor_path_parameter.flatten()
        initial_configuration = np.array([robot_position[0], robot_position[1], robot_orientation[0], governor_path_parameter[0]])


        governed_controller_dynamics = lambda t, X: robot_governor_vector_field(X, safety_gain, convergence_rate, robot_radius, OccGrid2D, path, **kwargs)
        tspan = [0, duration]

        sol = solve_ivp(governed_controller_dynamics, tspan, initial_configuration, **kwargs)

        return sol.t, sol.y

def robot_governor_discrete_update_ode45(robot_position, robot_orientation, governor_path_parameter, safety_gain, convergence_rate, robot_radius, OccGrid2D, path, duration, **kwargs):
        
        robot_position = np.array(robot_position)
        governor_path_parameter = np.array(governor_path_parameter)

        T, X = robot_governor_trajectory(robot_position, robot_orientation, governor_path_parameter, safety_gain, convergence_rate, robot_radius, OccGrid2D, path, duration, **kwargs)
        
        new_governor_parameter = X[3,-1]
        new_governor_position = path_value(path, new_governor_parameter).flatten()

        # make all kwarg keys lower case
        kwargs = {k.lower(): v for k, v in kwargs.items()}

        # get motion prediction function
        motion_prediction_fnc = kwargs['motion_prediction']
        motion_polygon = motion_prediction_fnc(robot_position, robot_orientation, new_governor_position,  kwargs['motion_prediction_args'])

        # get safety level
        frame = 'world'
        distance, XPolygon, XMap = OccGrid2D.distance_to_collision_polygon(motion_polygon, frame)
        safety_level = distance - robot_radius

        if safety_level < 0:
                return governor_path_parameter

        return new_governor_parameter