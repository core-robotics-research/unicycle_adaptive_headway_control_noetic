"""
Python Module for Path Tools typically used by the Time Governor
Author: Aykut Isleyen, a.isleyen@tue.nl
Created: May 2023
"""

import numpy as np

def path_length(path):
    """
    Computes the path length of points of a piecewise linear path with
    respect to the start point

    Usage:
        [L, L2] = path_length(path)
    Input:
        path - Piecewise linear path in n dimensional space, m x n matrix
    Output:
        L - Path length, 1 x 1 scalar
        L2 - Path length of points wrt the start, m x 1 matrix
    Example:
        path = np.random.rand(10,2)
        [L, L2] = path_length(path)
    """

    path = np.array(path)

    # Compute the path length L and L2 using cumulative sum
    L2 = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
    L2 = np.insert(L2, 0, 0)
    L = L2[-1]

    return L, L2


def path_value(path, k):
    """
    Computes the points of a unit-speed piecewise linear path at given path parameters

    Usage:
        Y = path_value(path, k)
    Input:
        path - Piecewise linear path points in n-dimensional space, m x n matrix
        k - Path parameters, 1 x l vector
    Output:
        Y - Path points, l x n matrix
    Example:
        path = np.random.rand(10,2)
        L, L2 = path_length(path)
        k = np.linspace(0, L, 20)
        k = k[np.random.permutation(len(k))]
        Y = path_value(path, k)
        plt.figure()
        plt.plot(path[:,0], path[:,1], 'k-')
        plt.scatter(Y[:,0], Y[:,1], c='r', marker='o')
        plt.show()
    """

    path = np.array(path)
    k = np.array(k)
    k = k.flatten()
    n = path.shape[1]
    Y = np.full((k.shape[0], n), np.nan)

    L, L2 = path_length(path)

    k[k < 0] = 0
    k[k > L] = L

    # If there are no points to compute, return
    if len(k) == 0:
        return Y

    # Find the indices of the segments that contain the points
    J = np.searchsorted(L2, k, side='right') - 1
    I = np.arange(len(k))
    
    # Append the last point to the path
    path = np.vstack((path, path[-1,:]))
    L2 = np.append(L2, L2[-1]+1e-10)

    # Compute the interpolation parameter
    w = (k[I] - L2[J]) / (L2[J+1] - L2[J])    
    # if w is not in between 0 and 1 or if it is nan, then set it to the closest value
    w[w < 0] = 0
    w[w > 1] = 1
    w[np.isnan(w)] = 0

    Y = path[J,:] + w[:,np.newaxis] * (path[J+1,:] - path[J,:])
    
    return Y


def path_distance(path, position):
    """
    Computes the distance of a point to a piecewise linear path

    Usage:
        distance, coords = path_distance(path, position)
    Input:
        path - Piecewise linear path points in n-dimensional space, m x n matrix
        position - Point in n-dimensional space, k x n vector
    Output:
        distance - Distance of the point to the path, k x 1 vector
        coords - Coordinates of the closest point on the path, k x n matrix
    Example:
        path = np.random.rand(10,2)
        position = np.random.rand(5,2)
        distance, coords = path_distance(path, position)
    """

    position = np.array(position)
    path = np.array(path)


    if path.shape[0] == 0:
            distance = np.full((position.shape[0],1), np.nan)
            coords = np.full((position.shape[0], path.shape[1]), np.nan)
    elif path.shape[0] == 1:
            distance = np.sqrt(np.sum((position - path)**2, axis=1))
            distance = distance.reshape((distance.shape[0],1))
            coords = np.tile(path, (position.shape[0], 1))
    else:

            distance = np.full((position.shape[0],1), np.nan)
            coords = np.full((position.shape[0], path.shape[1]), np.nan)

            Pbase = path[0:-1,:]
            Pdiff = np.diff(path, axis=0)
            Pmag2 = np.sum(Pdiff**2, axis=1)

            goal = position.copy()
            for k in range(position.shape[0]):
                    Gbase = position[k,:] - Pbase
                    Gmag2 = np.sum(Gbase**2, axis=1)

                    GProj = np.sum(Pdiff*Gbase, axis=1)
                    W = GProj/Pmag2
                    W = np.maximum(np.minimum(W, 1), 0)
                    D = (W**2)*Pmag2 - 2*W*GProj + Gmag2
                    I = np.where(D == np.min(D))[0]
                    I = I[-1]

                    distance[k] = np.sqrt(D[I])

                    w1 = GProj[I]/Pmag2[I]
                    w1 = np.maximum(np.minimum(w1, 1), 0)

                    coords[k,:] = w1*Pdiff[I,:] + Pbase[I,:]

    return distance, coords