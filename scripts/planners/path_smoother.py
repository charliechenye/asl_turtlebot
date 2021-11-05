import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    ########## Code starts here ##########
    # Hint 1 - Determine nominal time for each point in the path using V_des
    # Hint 2 - Use splrep to determine cubic coefficients that best fit given path in x, y

    path = np.array(path)
    x_old = path[:,0]
    y_old = path[:,1]
    n = np.shape(x_old)[0]
    t_segment = np.zeros((n,))
    t_segment[1:] = np.linalg.norm(np.column_stack((x_old[1:] - x_old[:-1], y_old[1:] - y_old[:-1])),
                                   axis = 1) / V_des
    t_path = np.cumsum(t_segment, dtype=float)
    x_trajectory = scipy.interpolate.splrep(t_path, x_old, s = alpha)
    y_trajectory = scipy.interpolate.splrep(t_path, y_old, s = alpha)

    t_smoothed = np.arange(0.0, max(t_path), dt)
    x_d = scipy.interpolate.splev(t_smoothed, x_trajectory)
    y_d = scipy.interpolate.splev(t_smoothed, y_trajectory)
    xd_d = scipy.interpolate.splev(t_smoothed, x_trajectory, der = 1)
    xdd_d = scipy.interpolate.splev(t_smoothed, x_trajectory, der = 2)
    yd_d = scipy.interpolate.splev(t_smoothed, y_trajectory, der = 1)
    ydd_d = scipy.interpolate.splev(t_smoothed, y_trajectory, der = 2)
    theta_d = np.arctan2(yd_d, xd_d)
    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return traj_smoothed, t_smoothed
