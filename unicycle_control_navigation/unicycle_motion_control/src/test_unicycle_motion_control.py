import numpy as np
import matplotlib.pyplot as plt
from unicycle_motion_control import *

#################################################################################################
############################ TEST ADAPTIVE HEADWAY CONTROL FUNCTIONS ############################
#################################################################################################

def test_ahc_controller(position, orientation, goal, epsgain, refgain, tol, motion_direction):
    

    v, w = adaptive_headway_ctrl(position, orientation, goal, epsgain, refgain, tol, motion_direction)
    
    print(v)
    print(w)

    return None

def test_ahc_vector_field(position, orientation, goal, epsgain, refgain, tol, motion_direction):

    dX = ahc_vector_field(position, orientation, goal, epsgain, refgain, tol, motion_direction)

    print(dX)

    return None

def test_ahc_trajectory_solver(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction):
    
    T, Y = ahc_trajectory(position, orientation, goal, [0, duration], epsgain, refgain, tol, motion_direction)

    fig, ax = plt.subplots()
    ax.plot(Y[:,0], Y[:,1], '-')
    ax.set_aspect('equal')
    plt.show()
    
    return None

def test_ahc_discrete_update(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction):

    position, orientation = ahc_discrete_update(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction)
    
    print(position)
    print(orientation)

    return None

def test_ahc_circular_motion_prediction(position, orientation, goal, epsgain, motion_direction, res):
    
    polygon = ahc_circle_motion_prediction(position, orientation, goal, epsgain, motion_direction, res)

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    ax.scatter(position[0], position[1], marker='o', color='b')
    ax.scatter(goal[0], goal[1], marker='o', color='r')

    T, Y = ahc_trajectory(position, orientation, goal, [0, 30], epsgain, refgain, tol, motion_direction)
    ax.plot(Y[:,0], Y[:,1], '-')

    plt.show()

    return None

def test_ahc_triangular_motion_prediction(position, orientation, goal, epsgain, motion_direction):

    polygon = ahc_triangle_motion_prediction(position, orientation, goal, epsgain, motion_direction)

    # print(polygon)

    fig, ax = plt.subplots()
    polygon = np.concatenate((polygon,  polygon[:1]), axis=0)
    ax.plot(polygon[:,0], polygon[:,1], '-')
    ax.set_aspect('equal')
    ax.scatter(position[0], position[1], marker='o', color='b')
    ax.scatter(goal[0], goal[1], marker='o', color='r')

    T, Y = ahc_trajectory(position, orientation, goal, [0, 30], epsgain, refgain, tol, motion_direction)
    ax.plot(Y[:,0], Y[:,1], '-')

    plt.show()

    return None


if __name__ == '__main__':
    
    position = [0 , 2]
    orientation = np.pi/4
    goal = [0, 0]


    epsgain = np.sqrt(2)/2
    refgain = 1.0
    tol = 1e-3
    motion_direction = 'bidirectional'
    duration = 5
    res = 60



#######################################################################################
############################ TEST ADAPTIVE HEADWAY CONTROL ############################
#######################################################################################
    # test_ahc_controller(position, orientation, goal, epsgain, refgain, tol, motion_direction)
    # test_ahc_vector_field(position, orientation, goal, epsgain, refgain, tol, motion_direction)
    # test_ahc_trajectory_solver(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction)
    # test_ahc_discrete_update(position, orientation, goal, duration, epsgain, refgain, tol, motion_direction)
    # test_ahc_circular_motion_prediction(position, orientation, goal, epsgain, motion_direction, res)
    # test_ahc_triangular_motion_prediction(position, orientation, goal, epsgain, motion_direction)