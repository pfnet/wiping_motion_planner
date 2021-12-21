import numpy as np
import math
import matplotlib.pyplot as plt
from utils import *

import utils as ut
import utils_plot as utp
# import Draw_on_Screen as DS

PI = 3.141592653589793




def velocity_profiler(path, T_final = []):
    '''
    Interactively draw the desired path. If you provide T_final, also interpolate
    :rtype: object
    :param T_final: optional final time
    :param path: current path
    :return: new time and position, where time is equally spaced
    '''

    if T_final == []:
        T_final = 1

    n_traj = path.shape[0]

    t = np.linspace(0, T_final, n_traj)
    draw = DS.Draw_on_Screen('velocity_profiler')
    utp.move_figure(draw.h['fig'], x=3080, y=0)
    plt.plot(t, path)


    if 1: # draw on top of dt
        path2, indx = draw.iterative_fix(path, 50, scale_pad_x = 1.5)

    t2 = (1.0*indx/indx[-1])*T_final

    h1 = utp.figure_2D_subplot_row('traj_old_vs_new', n_row=2)
    utp.set_subplot_active(h1['ax1'])
    plt.plot(  t,   path )
    plt.plot(  t2, path2)
    utp.set_subplot_active(h1['ax2'])
    plt.plot(  t,  ut.time_derivative(t,  path) )
    plt.plot(  t2, ut.time_derivative(t2, path2) )

    return t2, path2

def procrustes(X, Y, scaling=False, reflection='false'):
    """
    A port of MATLAB's `procrustes` function to Numpy.

    Procrustes analysis determines a linear transformation (translation,
    reflection, orthogonal rotation and scaling) of the points in Y to best
    conform them to the points in matrix X, using the sum of squared errors
    as the goodness of fit criterion.

        d, Z, [tform] = procrustes(X, Y)

    Inputs:
    ------------
    X, Y
        matrices of target and input coordinates. they must have equal
        numbers of  points (rows), but Y may have fewer dimensions
        (columns) than X.

    scaling
        if False, the scaling component of the transformation is forced
        to 1

    reflection
        if 'best' (default), the transformation solution may or may not
        include a reflection component, depending on which fits the data
        best. setting reflection to True or False forces a solution with
        reflection or no reflection respectively.

    Outputs
    ------------
    d
        the residual sum of squared errors, normalized according to a
        measure of the scale of X, ((X - X.mean(0))**2).sum()

    Z
        the matrix of transformed Y-values

    tform
        a dict specifying the rotation, translation and scaling that
        maps X --> Y

    """

    n,m = X.shape
    ny,my = Y.shape

    muX = X.mean(0)
    muY = Y.mean(0)

    X0 = X - muX
    Y0 = Y - muY

    ssX = (X0**2.).sum()
    ssY = (Y0**2.).sum()

    # centred Frobenius norm
    normX = np.sqrt(ssX)
    normY = np.sqrt(ssY)

    # scale to equal (unit) norm
    X0 /= normX
    Y0 /= normY

    if my < m:
        Y0 = np.concatenate((Y0, np.zeros(n, m-my)),0)

    # optimum rotation matrix of Y
    A = np.dot(X0.T, Y0)
    U,s,Vt = np.linalg.svd(A,full_matrices=False)
    V = Vt.T
    T = np.dot(V, U.T)

    if reflection is not 'best':

        # does the current solution use a reflection?
        have_reflection = np.linalg.det(T) < 0

        # if that's not what was specified, force another reflection
        if reflection != have_reflection:
            V[:,-1] *= -1
            s[-1] *= -1
            T = np.dot(V, U.T)

    traceTA = s.sum()

    if scaling:

        # optimum scaling of Y
        b = traceTA * normX / normY

        # standarised distance between X and b*Y*T + c
        d = 1 - traceTA**2

        # transformed coords
        Z = normX*traceTA*np.dot(Y0, T) + muX

    else:
        b = 1
        d = 1 + ssY/ssX - 2 * traceTA * normY / normX
        Z = normY*np.dot(Y0, T) + muX

    # transformation matrix
    if my < m:
        T = T[:my,:]
    c = muX - b*np.dot(muY, T)

    #transformation values
    tform = {'rotation':T, 'scale':b, 'translation':c}

    return d, Z, tform


def interpT(T1, T2, nSteps):
    # function Tout = interpT(T1, T2, nSteps)
    #
    # INPUT
    #   T1 = [4x4] initial pose as homog. trasnf. matrices.
    #   T2 = [4x4] final pose as homog. trasnf. matrices.
    #   nSteps = number of steps between Ts and Tf
    #
    # OUTPUT
    #   Trajectory shape=(nSteps, 4, 4)
    #

    # from T to quaternion
    xyzq1 = T2xyzq(T1)
    xyzq2 = T2xyzq(T2)

    # do slerp here
    qint = slerp(xyzq1[3:], xyzq2[3:], nSteps)

    # interpolate xyz here
    # note that on purpose, I make time steps as columns, which is not usual
    xyzint = np.zeros(shape=(7, nSteps))
    for j in range(3):
        #a = np.linspace(xyzq1[j], xyzq2[j], nSteps)[:,0]
        xyzint[j] = np.linspace(xyzq1[j], xyzq2[j], nSteps)[:,0]

    for k, j in zip(range(0, 4), range(3, 7)):
        #print(k, j)
        xyzint[j] = qint[k]

    return xyzq2T(xyzint)


def get_xyz_from_T(T):
    if type(T) is list:
        T = np.array(T)

    if T.ndim == 2:
        return T[0:3, 3]

    else:
        xyz = np.zeros(shape=(3,T.shape[0]))
        for t in xrange(T.shape[0]):
            xyz[:,t] = T[t, 0:3, 3]
        return xyz


def rotate_T_around_rot_axis(Tin, val, axis_location, rotation_axis='z', format='deg'):
    '''
    Rotate a point around a translated rotation axis.
    :param Tin: the homog transf matrix of the point. Tin is not a trajectory but a single point
    :param val: the amount of rotation around the axis
    :param axis_location: a tuple whose interpretation depends on the chosen rotation axis
            if rotation axis is x
                (y, z): the coordinates of the point where the rotation axis crosses the yz plane
            if rotation axis is y
                (x, z): the coordinates of the point where the rotation axis crosses the xz plane
            if rotation axis is z
                (x, y): the coordinates of the point where the rotation axis crosses the xy plane
    :param rotation_axis: 'x, y or z'
    :param type:
    :param format:
    :return:
    '''

    # move Tin such that axi_location is the origin
    # delta = Tin[0:3,-1] - axis_location

    Tin = np.copy(Tin)

    if format is 'deg':
        deg = 1
    else:
        deg = 0

    T = np.zeros(shape=(4, 4))
    if rotation_axis is 'z':
        R = rotz(val, deg)
        Tin[0, -1] = Tin[0, -1] - axis_location[0]
        Tin[1, -1] = Tin[1, -1] - axis_location[1]

    if rotation_axis is 'y':
        R = roty(val, deg)
        #T[0, -1] = axis_location[0]
        #T[2, -1] = axis_location[1]
        print("IMPLEMENT THIS BETTER")
        raw_input()

    if rotation_axis is 'x':
        R = rotx(val, deg)
        Tin[1, -1] = Tin[1, -1] - axis_location[0] # y
        Tin[2, -1] = Tin[2, -1] - axis_location[1] # z

    Trot = np.eye(4)
    Trot[0:3, 0:3] = R

    Trot = Trot + T

    # delta between axes
    Tin[0:3, -1] = Tin[0:3, -1] - Trot[0:3, -1]

    # this is extrisic only
    Tout = np.matmul(Trot, Tin)

    if rotation_axis is 'z':
        Tout[0, -1] = Tout[0, -1] + axis_location[0]
        Tout[1, -1] = Tout[1, -1] + axis_location[1]


    if rotation_axis is 'x':
        Tout[1, -1] = Tout[1, -1] + axis_location[0]
        Tout[2, -1] = Tout[2, -1] + axis_location[1]



    return Tout


# def rotate_T_around_rot_axis(Tin, val, axis_location, rotation_axis = 'z', format = 'deg'):
#     '''
#     Rotate a point around a translated rotation axis.
#     :param Tin: the homog transf matrix of the point. Tin is not a trajectory but a single point
#     :param val: the amount of rotation around the axis
#     :param axis_location: a tuple whose interpretation depends on the chosen rotation axis
#             if rotation axis is x
#                 (y, z): the coordinates of the point where the rotation axis crosses the yz plane
#             if rotation axis is y
#                 (x, z): the coordinates of the point where the rotation axis crosses the xz plane
#             if rotation axis is z
#                 (x, y): the coordinates of the point where the rotation axis crosses the xy plane
#     :param rotation_axis: 'x, y or z'
#     :param type:
#     :param format:
#     :return:
#     '''
#
#
#     # move Tin such that axi_location is the origin
#     # delta = Tin[0:3,-1] - axis_location
#     # Tin[0:3,-1]  =  Tin[0:3,-1] - axis_location
#
#     if format is 'deg':
#         deg = 1
#     else:
#         deg = 0
#
#     T = np.zeros(shape=(4,4))
#     if rotation_axis is 'z':
#         R = rotz(val, deg)
#         T[0:2,-1] = axis_location
#
#     if rotation_axis is 'y':
#         R = roty(val, deg)
#         T[0, -1] = axis_location[0]
#         T[2, -1] = axis_location[1]
#
#     if rotation_axis is 'x':
#         R = rotx(val, deg)
#         T[1, -1] = axis_location[0]
#         T[2, -1] = axis_location[1]
#
#     Trot = np.eye(4)
#     Trot[0:3, 0:3] = R
#
#     Trot = Trot + T
#
#
#     # delta between axes
#     Tin[0:3,-1] =  Tin[0:3,-1]-Trot[0:3,-1]
#
#     # this is extrisic only
#     Tout = np.matmul(Trot, Tin)
#
#
#     return Tout

def rotate_T(Tin, val, rotation_axis = 'z', type ='extrinsic', format = 'deg'):
    '''
    Rotate a point around itself. This can be intrinsic or extrinsic depending on the type you used.
    :param Tin:
    :param val:
    :param rotation_axis:
    :param type:
    :param format:
    :return:
    '''

    # rotate on its own axis, by moving the input to the origin of the frame.
    xyz = np.copy(Tin[0:3,-1])
    Tin[0:3, -1] = [0,0,0]


    if format is 'deg':
        deg = 1
    else:
        deg = 0

    if rotation_axis is 'z':
        R = rotz(val, deg)
    if rotation_axis is 'y':
        R = roty(val, deg)
    if rotation_axis is 'x':
        R = rotx(val, deg)

    Trot = np.eye(4)
    Trot[0:3, 0:3] = R

    if type is 'extrinsic':
        Tout = np.matmul(Trot, Tin)

    if type is 'intrinsic':
        Tout = np.matmul(Tin, Trot)

    Tout[0:3, -1] = xyz

    return Tout


def rotz(t, deg=False):

    if deg:
        t = t * PI / 180

    ct = math.cos(t)
    st = math.sin(t)
    R = [
        [ct, -st, 0],
        [st, ct, 0],
        [0, 0, 1]
    ]
    return R


def roty(t, deg=0):
    if deg:
        t = t * PI / 180

    ct = math.cos(t)
    st = math.sin(t)
    R = [
        [ct, 0, st],
        [0, 1, 0],
        [-st, 0, ct]
    ]
    return R


def rotx(t, deg=0):
    if deg:
        t = t * PI / 180

    ct = math.cos(t)
    st = math.sin(t)
    R = [
        [1, 0, 0],
        [0, ct, -st],
        [0, st, ct]
    ]
    return R

def Tdelta(T0, T1):
    R0 = T0[0:3, 0:3]
    R1 = T1[0:3, 0:3]
    Rmult = np.matmul(R1,R0.T)
    vex_ = vex( Rmult - np.eye(3) )

    delta = np.array((T1[0:3,3]-T0[0:3,3], vex_[0:]))
    return delta.flatten()



def vex(S):
    v = 0.5*np.array([ S[2,1]-S[1,2], S[0,2]-S[2,0], S[1,0]-S[0,1] ])
    return v

def homogTransfPlot(T, ax, nPlot=20, scale=0.085, shadow=0,
                    rgb_light=0, plot_path=1, path_color=[0.6, 0.6, 0.6], path_line_width=1, forceDraw=0,
                    ref_frame_line_width=1):
    art = []
    if T.ndim == 2:  # there is a single point as a 2 dimensional array, and not a trajectory
        art.append(plot_coord_frame(T, ax, scale, shadow, rgb_light, ref_frame_line_width))
    else:
        nTraj = (T.shape)[0]

        rangee = (np.linspace(0, nTraj - 1, nPlot)).astype(int)
        for t in rangee:
            art.append(plot_coord_frame(T[t], ax, scale, shadow, rgb_light, ref_frame_line_width))

        xyz = np.zeros((T.shape[0], 3))
        for k in range(0, T.shape[0]):
            xyz[k, :] = get_xyz_from_T(T[k])

        art.append(ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color=path_color, linewidth=path_line_width))
        if shadow:
            art.append(ax.plot(xyz[:, 0], xyz[:, 1], shadow * np.ones(shape=(nTraj)), color=[.6, .6, .6],
                               linewidth = 1))

    if forceDraw:
        update_plot(ax)

    # flatten the list to 1D
    artflat = [j for i in art for j in i]
    return artflat


def plot_coord_frame(T, ax, scale, shadow, rgb_light, ref_frame_line_width):
    if rgb_light == 0:
        rgb = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    else:
        rgb = [[1, 0.5, 0.5], [0.5, 1, 0.5], [0.5, 0.5, 1]]

    ref_frame_xyz = [[scale, 0, 0, 1], [0, scale, 0, 1], [0, 0, scale, 1]]
    orig = np.matmul(T, [0, 0, 0, 1])[0:3]

    artist = []
    for j in [0, 1, 2]:
        outx1 = orig
        outx2 = np.matmul(T, ref_frame_xyz[j])[0:3]
        a1, = ax.plot([outx1[0], outx2[0]], [outx1[1], outx2[1]], [outx1[2], outx2[2]], label='parametric curve',
                      color=rgb[j], linewidth=ref_frame_line_width)
        artist.append(a1)

        # maybe should plot shadow of reference frame ??
        # if shadow:
        #     ax.plot([outx1[0], outx2[0]], [outx1[1], outx2[1]], [shadow, shadow], label='parametric curve',
        #             color=[0.6, 0.6, 0.6], linewidth=1)

    return artist


def xyzq2T(xyzq):
    '''
    You can use an entire trajectory.
    If only a single value, then use
        T = kinematics.xyzq2T(xyzq)[0]
    '''


    if type(xyzq) is list:
        xyzq = np.array(xyzq)

    T_new = []

    # single time step trajectories
    if xyzq.shape[0] ==7:
        try:
            xyzq.shape[1]
        except:
            xyzq = np.array([xyzq]).T


    # check this is transposed or not
    if xyzq.shape[0] == 7:
        xyzq = xyzq.T

    for t in range(0, xyzq.shape[0]):
        T_new.append(xyzq2T_single(xyzq[t, :]))

    T_new = np.array(T_new)
    return T_new



def T2xyzq(T):
    '''
    If T is (4,4) then you have to use
        kinematics.T2xyzq(T).T[0]
    '''
    quat = R2quat(T)
    lst = [T[0, 3], T[1, 3], T[2, 3], quat[0], quat[1], quat[2], quat[3]]
    xyzq = np.zeros(shape=(7, 1))
    xyzq[:, 0] = lst
    return xyzq



def quat2euler(quat):

    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]


    eps_ = 0.00

    #quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z) + eps_

    t1 = +1.0 - 2.0 * (x * x + y * y)  + eps_

    if 0:
        if t1 > 0 and t1 < 0.1:
            print('++++++++++')
            t1 = 0.1
        if t1 < 0 and t1 > -0.1:
            print('-------')
            t1 = -0.1

    roll = math.atan2(t0, t1)
    #print('t0, t1, roll: %5.3f, %5.3f, %5.3f' %(  t0, t1, roll )  )





    t2 = +2.0 * (w * y - z * x)  + eps_
    t2 = +1.0 if t2 > +1.0 else t2  + eps_
    t2 = -1.0 if t2 < -1.0 else t2  + eps_
    pitch = math.asin(t2)


    t3 = +2.0 * (w * z + x * y) + eps_
    t4 = +1.0 - 2.0 * (y * y + z * z) + eps_
    yaw = math.atan2(t3, t4)



    return [yaw, pitch, roll]


def euler2quat(ypr):

    yaw   = ypr[0]
    pitch = ypr[1]
    roll  = ypr[2]

    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qw, qx, qy, qz]


def R2quat(t):
    # TR2Q   Convert homogeneous transform to a unit-quaternion
    #
    #   Q = tr2q(T)
    #
    #   Have to check that t can also be only the R[3,3] of T
    #
    #   Return a unit quaternion corresponding to the rotational part of the
    #   homogeneous transform T.

    if 0:  # == 3: # only rotation was given, so I should make it 4x4 because the trace is being used somewhere??
        t = np.concatenate((t, [np.array([0, 0, 0])]))
        colzeros = np.array([[0, 0, 0, 1]])
        t = np.concatenate((t, colzeros.T), axis=1)

    aux_term = np.trace(t[0:3, 0:3])
    if  aux_term  < -1:
        aux_term = -1+0.000001

    qs = math.sqrt(  aux_term   + 1)    /    2.0
    kx = t[2, 1] - t[1, 2]  # Oz - Ay
    ky = t[0, 2] - t[2, 0]  # Ax - Nz
    kz = t[1, 0] - t[0, 1]  # Ny - Ox

    if (t[0, 0] >= t[1, 1]) and (t[0, 0] >= t[2, 2]):
        kx1 = t[0, 0] - t[1, 1] - t[2, 2] + 1  # Nx - Oy - Az + 1
        ky1 = t[1, 0] + t[0, 1]  # Ny + Ox
        kz1 = t[2, 0] + t[0, 2]  # Nz + Ax
        add = (kx >= 0)
    elif (t[1, 1] >= t[2, 2]):
        kx1 = t[1, 0] + t[0, 1]  # Ny + Ox
        ky1 = t[1, 1] - t[0, 0] - t[2, 2] + 1  # Oy - Nx - Az + 1
        kz1 = t[2, 1] + t[1, 2]  # Oz + Ay
        add = (ky >= 0)
    else:
        kx1 = t[2, 0] + t[0, 2]  # Nz + Ax
        ky1 = t[2, 1] + t[1, 2]  # Oz + Ay
        kz1 = t[2, 2] - t[0, 0] - t[1, 1] + 1  # Az - Nx - Oy + 1
        add = (kz >= 0)

    if add:
        kx = kx + kx1
        ky = ky + ky1
        kz = kz + kz1
    else:
        kx = kx - kx1
        ky = ky - ky1
        kz = kz - kz1

    kv = np.matrix([kx, ky, kz])
    nm = np.linalg.norm(kv)
    if nm == 0:
        qout = [1, 0, 0, 0]
    else:
        q_ = (math.sqrt(1 - qs ** 2) / nm) * kv
        qout = [qs, q_[0, 0], q_[0, 1], q_[0, 2]]

    return qout


def xyzq2T_single(xyzq):
    
    R = quat2R(xyzq[3:])
    T = np.concatenate((R, [ [xyzq[0]], [xyzq[1]], [xyzq[2]]]   ), axis=1)
    T = np.concatenate((T, [[0, 0,0,1]]  ), axis=0)

    return T


def quat2R(q):
    s = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    r = [[1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - s * z), 2 * (x * z + s * y)],
         [2 * (x * y + s * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - s * x)],
         [2 * (x * z - s * y), 2 * (y * z + s * x), 1 - 2 * (x ** 2 + y ** 2)]
         ]

    return np.array(r)


def slerp(q1, q2, nTraj):
    traj = np.linspace(0, 1, nTraj)

    qint = np.zeros(shape=(4, nTraj))
    for k in range(nTraj):
        qint[:,k] = slerp_unit(q1, q2, traj[k])

    return qint


# This was stripped from Peter Corke's toolbox python implementation Quaternion class
def slerp_unit(q1, q2, r):


    if type(q1) is list:
        q1 = np.array(q1)
    if type(q2) is list:
        q2 = np.array(q2)

    cosTheta = np.matmul(q1.T, q2)
    if cosTheta < 0:
        q1 = - q1
        cosTheta = - cosTheta

    theta = np.arccos(cosTheta)

    # clip values of  r
    if r < 0:
        print('interpolation index must be between 0 and 1')
        quit()
    if r > 1:
        print('interpolation index must be between 0 and 1')
        quit()

    q = np.zeros(shape=(4, 1))
    if theta == 0:
        q[:, ] = q1
    else:
        q[:, ] = (np.sin((1 - r) * theta) * q1 + np.sin(r * theta) * q2) / np.sin(theta)


    return q[:,0]

def test_delta_on_T():

    T1 = np.eye(4)
    T2 = np.eye(4)

    T1[0:3, 0:3] = np.matmul(T1[0:3, 0:3], (rotz(10, 1)))
    T1[0:3, 0:3] = np.matmul(T1[0:3, 0:3], (roty(-10, 1)))
    T1[0:3, 0:3] = np.matmul(T1[0:3, 0:3], (rotx(-30, 1)))

    T2[0:3, 0:3] = np.matmul(T2[0:3, 0:3], (rotz(90, 1)))
    T2[0:3, 0:3] = np.matmul(T2[0:3, 0:3], (roty(45, 1)))
    T2[0:3, 0:3] = np.matmul(T2[0:3, 0:3], (rotx(-30, 1)))

    T2[0:3, -1] = [0.1, 0.3, -0.20]

    Tdelta(T1, T2)


def test_procrustes(test_data):

    xyzL = test_data.xyzqleft[0:3, :]
    xyzR = test_data.xyzqright[0:3, :]

    d, Z, tform = procrustes(xyzL.T, xyzR.T)
    return d, Z, tform


def test_quaternion_interp(test_data):
    T1 = np.eye(4)
    T2 = np.eye(4)

    T1[0:3, 0:3] = np.matmul(T1[0:3, 0:3], (rotz(0*10, 1)))
    T1[0:3, 0:3] = np.matmul(T1[0:3, 0:3], (roty(-0*10, 1)))
    T1[0:3, 0:3] = np.matmul(T1[0:3, 0:3], (rotx(-0*30, 1)))

    T2[0:3, 0:3] = np.matmul(T2[0:3, 0:3], (rotz(0*90, 1)))
    T2[0:3, 0:3] = np.matmul(T2[0:3, 0:3], (roty(0*180, 1)))
    T2[0:3, 0:3] = np.matmul(T2[0:3, 0:3], (rotx(0*-90, 1)))

    T2[0:3, 0:3] = np.matmul((rotz(90, 1)), T2[0:3, 0:3])

    T2[0:3, -1] = [0.1, 0.3, -0.20]

    T = np.zeros(shape=(2, 4, 4))
    T[0] = T1
    T[1] = T2

    h = {}
    h['fig2'], h['ax2'], h['name2'] = plot_workspace('test6')
    homogTransfPlot(T, h.get('ax2'), scale=0.125, ref_frame_line_width=3, forceDraw=1)


    if 0:
        # from T to quaternion
        xyzq1 = T2xyzq(T1)
        xyzq2 = T2xyzq(T2)

        # do slerp here
        qint = slerp(xyzq1[3:], xyzq2[3:], 20)

        # interpolate xyz here
        xyzint = np.zeros(shape=(7,20))
        for j in xrange(3):
            xyzint[j] = np.linspace(xyzq1[j], xyzq2[j], 20)

        for k, j in zip(range(0, 4), range(3, 7)):
            print(k,j)
            xyzint[j] = qint[k]

        homogTransfPlot(xyzq2T(xyzint), h.get('ax2'), scale=0.125, ref_frame_line_width=1, forceDraw=1, rgb_light=1)

    Tint = interpT(T1, T2, 20)
    homogTransfPlot( Tint , h.get('ax2'), scale=0.125, ref_frame_line_width=1, forceDraw=1, rgb_light=1)



def plot_workspace(name_of_plot):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D, proj3d
    # proj3d.persp_transformation = orthogonal_proj

    fig = plt.figure(name_of_plot)
    ax = fig.add_subplot(111, projection='3d')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))

    plt.ion()
    fig.show()

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    # ax.set_aspect('equal')
    # ax.auto_scale_xyz(np.array([-1, 1])*0.5, np.array([-1, 1])*0.5, np.array([-1, 1])*0.5)

    return fig, ax, name_of_plot


def velocity_profiler_DELETABLE(t, pos):
    '''
    Interactively draw the desired warping of time. This will change the velocity profile of a given path
    :param t: current time profile
    :param pos: current path
    :return: new time and position, where time is equally spaced
    '''

    draw = DS.Draw_on_Screen('velocity_profiler')
    utp.move_figure(draw.h['fig'], x=10, y=10)
    plt.plot(t, pos)

    if 0: # draw time directly
        z =  draw.iterative_fix(t, 7)
        z_dot = np.diff(z)
        min_dt =  np.abs(z_dot).min()
        z_dot[z_dot < 0] = min_dt

    if 1: # draw on top of dt
        zdot = draw.iterative_fix(np.diff(t), 7)

        z_dot = zdot
    zfix = np.cumsum(z_dot)

    h1 = utp.figure_2D_list('newtime')
    plt.plot(zfix, color=utp.color('k', 0.01))
    plt.plot(t, color=utp.color('b', 0.01))


    h1 = utp.figure_2D_list('asd')
    plt.plot(zfix, range(zfix.shape[0]) , 'o' )

    zfix_lin = np.linspace(zfix[0], zfix[-1], zfix.shape[0])
    id_nl  =np.interp( zfix_lin,  zfix, range(zfix.shape[0]) )

    h1 = utp.figure_2D_list('pos_new')
    plt.plot(  zfix_lin,   pos[id_nl.astype(int)]  )
    plt.plot(  zfix, pos[:-1])
    plt.plot(t, pos)

    h1d = utp.figure_2D_list('vel_new')
    plt.plot(  zfix_lin,  ut.time_derivative(zfix_lin,  pos[id_nl.astype(int)] ) )
    plt.plot(  zfix, ut.time_derivative(zfix, pos[:-1]))
    plt.plot(  t, ut.time_derivative(t, pos))

    return tnew, posnew
