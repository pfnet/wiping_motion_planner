

import matplotlib.pyplot as plt
import numpy as np
import utils_plot as utp
import yaml
import matplotlib.path as mpltPath

try:
    import cv2
except:
    cv2 = []


def load_yaml(yaml_file_location):


    with open(yaml_file_location) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)


    return data


def resample_based_on_traveled_distance(states_for_distance, X, debug = False,
                                        profile = 'trapez',
                                        trapez_ratio = [.25, .5, .25]):
    '''
    Resamples a trajectory uniformly such that a path ends up having constant velocity.
    If enforce_zero_vel is True, it will also enforce that the start and end of the trajectories have zero velocity.

    Imposes zero velocities at start and end of a path.

    :param states_for_distance: [n_traj, n_variable] we use these states to compute the "distance traveled" by the robot.
                                "distance traveled" depends on the task.
                                For a mobile robot whose main task is to use the arms (e.g. for grasping) maybe it makes sense to
                                use the end-effector xyz positions to compute the distance traveled.
                                On the other hand, if the main task is about navigation of the base and the arms do not move much, perhaps the distance
                                traveled could be the xy coordinates of the base.
                                For this reason, the number of columns in the function can be variable.

    :param X: [n_traj, n_DoF] is a N-dim trajectory where each dimension is going to be re-sampled indepedently (except that they use the same odom)
    :param profile. Set the velocity profile of the trajectory
                    'sine':   smooth sinusoidal where max velocity is at the middle
                    'trapez': [25% accelerate 50% coast  25% decelerate]
                    'none':   does linear interpolation, as such the trajectories are not guaranteed to have
                              zero velocities at the start and end.
    :return:
    '''

    if len(states_for_distance.shape) == 1:
        states_for_distance = states_for_distance[:,np.newaxis]

    if len(X.shape) == 1:
        X = X[:,np.newaxis]


    n_traj, n_states = states_for_distance.shape
    diff_states = np.diff(states_for_distance, axis=0)

    sum_ = 0
    for j in range(n_states):
        sum_ = sum_ + diff_states[:, j] ** 2
    delta = np.sqrt(sum_)
    travel = np.cumsum(delta)
    travel = np.hstack(([0], travel))


    odom = travel[:]
    amp  = odom[-1] - odom[0]

    if debug:
        utp.figure_2D_list('dist')
        plt.plot(odom)

    # ideal cummulative travel distance based on smooth movement
    if profile == 'sine': # good for robot trajectories
        t      = np.linspace(0, 2 * np.pi * 0.5, len(odom))
        ideal  = amp * (-0.5 * np.cos(t) + 0.5)

    elif profile == 'trapez':

        vel = np.concatenate((np.zeros(3), np.linspace(0,1, int(n_traj * trapez_ratio[0])),
                              np.linspace(1,1, int(n_traj * trapez_ratio[1])),
                              np.linspace(1,0, int(n_traj * trapez_ratio[2])),
                              np.zeros(3)))


        vel   = resample(n_traj, vel)
        ideal = np.cumsum(vel)
        ideal = amp*(ideal/ideal[-1])


    else: # does linear distribution of points along the trajectory, but cannot enforce zero vel at the edges.
        ideal = np.linspace(0, amp, len(odom))

    if debug:
        utp.figure_2D_list('ideal_profile')
        plt.plot(ideal, '.')

    X2 = np.empty(X.shape)

    n_dof = X2.shape[1]

    for j in range(n_dof):
        X2[:,j] = interp(ideal, odom, X[:, j])


    if debug:
        utp.figure_2D_list('ideal_profile')
        plt.plot(ideal, '.')
        utp.figure_2D_list('ideal_profile_vel')
        plt.plot(np.diff(ideal), '.')

        utp.figure_2D_list('q')
        plt.plot(X, linewidth=1)
        plt.plot(X2, linewidth=3)

        utp.figure_2D_list('qd')
        plt.plot(np.diff(X, axis=0), linewidth=1)
        plt.plot(np.diff(X2, axis=0), linewidth=3)


    return X2


def smooth_trajectory(odom, X, debug = False, enforce_zero_vel = True):
    '''
    Imposes zero velocities at start and end of a path.
    :param odom: is the cumulative distasmooth_simplence traveled on the Euclidan space. For example, for a trajectory in Cartesian space
                 odom can be computed as
                    diff_xyz = np.diff(xyzq[:, 0:3], axis=0)
                    odom = np.sqrt(diff_xyz[:, 0] ** 2 + diff_xyz[:, 1] ** 2 + diff_xyz[:, 2] ** 2)
                    odom = np.cumsum(odom)
                    odom = np.hstack(([0], odom))
    :param X: [n_traj, n_DoF] is a N-dim trajectory where each dimension is going to be re-sampled indepedently (except that they use the same odom)
    :param enforce_zero_vel. Set to False to create a fixed step size along the trajectory rather than
                             imposing zero velocity.
    :return:
    '''

    amp = odom[-1] - odom[0]

    if debug:
        utp.figure_2D_list('dist')
        plt.plot(odom)


    # ideal cummulative travel distance based on smooth movement
    if enforce_zero_vel: # good for robot trajectories
        t = np.linspace(0, 2 * np.pi * 0.5, len(odom))
        ideal = amp * (-0.5 * np.cos(t) + 0.5)
    else: # does linear distribution of points along the trajectory, but cannot enforce zero vel at the edges.
        ideal = np.linspace(0, amp, len(odom))


    X2 = np.empty(X.shape)

    n_dof = X2.shape[1]

    for j in range(n_dof):
        X2[:,j] = interp(ideal, odom, X[:, j])


    if debug:
        utp.figure_2D_list('ideal_profile')
        plt.plot(ideal, '.')

        utp.figure_2D_list('q')
        plt.plot(X, linewidth=1)
        plt.plot(X2, linewidth=3)

        utp.figure_2D_list('qd')
        plt.plot(np.diff(X, axis=0), linewidth=1)
        plt.plot(np.diff(X2, axis=0), linewidth=3)


    return X2

def intersecting_2D_segments(path2, path1):

    rad_= 0.005
    path    = mpltPath.Path(path1)
    inside2 = path.contains_points(path2)

    idx_sol1 = []
    [idx_sol1.append(i) for i, x in enumerate(inside2) if x]

    if notempty(idx_sol1):
        idx_sol2 = []
        [idx_sol2.append(i) for i, x in enumerate(inside2) if not x]

        path1_sol1 = path2[idx_sol1, :]
        path1_sol2 = path2[idx_sol2, :]

        sol1 = {'xy': path1_sol1, 'idx': idx_sol1}
        sol2 = {'xy': path1_sol2, 'idx': idx_sol2}

        if isempty(sol1['idx']): sol1 = []
        if isempty(sol2['idx']): sol2 = []
        return sol1, sol2
    else:
        return [], []



def resample(n_traj, f_old):
    # resample via linear interpolation

    n_traj_old = f_old.shape[0]
    t_new  = np.linspace(0, 1, n_traj)
    t_old  = np.linspace(0, 1, n_traj_old)

    return  interp(t_new, t_old, f_old)

def resample_n_dim(n_traj, f_old):
    '''
    Do a resample on a nDof data.
    Assume rows are time steps
    Assume columns are degrees of freedom
    :param n_traj:
    :param f_old:
    :return:
    '''

    n_dof = f_old.shape[1]

    f_new = np.empty(shape=(n_traj, n_dof))

    for j in range(n_dof):
        f_new[:, j] = resample(n_traj, f_old[:, j])


    return f_new


def interp(t_new, t_old, f_old):

    if f_old.shape[0] == f_old.size:
        f_new = np.interp(t_new, t_old, f_old)

    else:
        n_dim = f_old.shape[1]
        f_new = np.empty(shape=(t_new.shape[0], n_dim))

        for j in range(n_dim):
            f_new[:, j] = np.interp(t_new, t_old, f_old[:,j])


    return f_new

def isempty(q):

    if (q!= []) is False:
        return True
    else:
        return False

def notempty(q):

    if (q!= []) is False:
        return False
    else:
        return True

def string_date_time():

    import datetime
    now = datetime.datetime.now()
    stringy = '%d' % now.year + '%02d' % now.month  + '%02d' % now.day+ '_' + '%02d' % now.hour  + '%02d' % now.minute + '%02d' % now.second

    return stringy


def r2d(x):
    if type(x) is list:
        x = np.array(x)

    return 360/(2*PI)*x

def d2r(x):
    if type(x) is list:
        x = np.array(x)

    return x*(2*PI)/360
#

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)
#
def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def smooth_simple(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop  ))

