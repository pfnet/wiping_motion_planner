from __future__ import print_function

import utils as ut
import utils_plot as utp
import numpy as np
import matplotlib.pyplot as plt


def create_initial_cleaning_path(prm_wipe):

    def uniform_step_size(x,y,n_traj):
        xy = np.array([x,y]).T
        xy = ut.resample_n_dim(n_traj, xy)

        diff_xy = np.diff(xy[:, 0:2], axis=0)
        odom = np.sqrt(diff_xy[:, 0] ** 2 + diff_xy[:, 1] ** 2)
        odom = np.cumsum(odom)
        odom = np.hstack(([0], odom))
        return ut.smooth_trajectory(odom, xy, enforce_zero_vel=False)

    def long_stroke_whipe(step_size, amplitude, n_traj=100):
        t = np.linspace(0, 2 * np.pi, n_traj)
        x = amplitude + amplitude * (-np.cos(t + np.pi * .0))
        y = step_size + step_size * (-np.cos(t * .5))
        x_new, y_new = -y, x
        return x_new, y_new

    user_selected_table_type = prm_wipe['in_use']['table_type']
    user_selected_wipe_type  = prm_wipe['in_use']['wipe_type']
    wipe_motion_params = prm_wipe[user_selected_table_type]['wipe_pattern'][user_selected_wipe_type]

    if user_selected_wipe_type == 'inside_out':
        n_traj = wipe_motion_params['n_traj']
        x_amp  = wipe_motion_params['x_amp']
        y_amp = wipe_motion_params['y_amp']

        t = np.linspace( (2+5)*np.pi, (0+5)*np.pi, n_traj)
        x = -x_amp*( t * np.cos(t))
        y =  y_amp*( t * np.sin(t))
        x,y = x-x[0],y-y[0]
        x_new, y_new = x, y

    if user_selected_wipe_type == 'long_stroke':
        x_new, y_new = long_stroke_whipe(
                                 step_size = wipe_motion_params['step_size']  ,
                                 amplitude = wipe_motion_params['amplitude']   ,
                                 n_traj    = wipe_motion_params['n_traj'] )

    if user_selected_wipe_type == 'long_stroke_with_circles':
        # long stroke whiping motion
        x_slow, y_slow = long_stroke_whipe(
                                 step_size = wipe_motion_params['step_size']  ,
                                 amplitude = wipe_motion_params['amplitude']   ,
                                 n_traj    = wipe_motion_params['n_traj'] )

        # add small circular motions
        amplitude_x = wipe_motion_params['circle_amplitude_x']
        amplitude_y = wipe_motion_params['circle_amplitude_y']
        n_circles_per_pass = wipe_motion_params['n_circles_per_pass']

        n_traj = wipe_motion_params['n_traj']
        t = np.linspace(0, 2*np.pi, n_traj)
        y = 0.5*np.cos(t)
        x = 0.5*np.sin(t)

        x = (x - x[0]) * amplitude_x
        y = (y - y[0]) * amplitude_y

        for _ in range(n_circles_per_pass):
            x = np.concatenate((x,x))
            y = np.concatenate((y,y))

        xy       = uniform_step_size(x, y, n_traj)
        xy_slow  = uniform_step_size(x_slow, y_slow, n_traj)

        xy_full = xy_slow + xy
        x_new, y_new = xy_full[:, 0], xy_full[:, 1]

    return x_new, y_new


def process_obstacle(xy, n=100, scale = [.5, .5], xy_shift = [0,0], n_smooth=3):
    '''

    :param xy:
    :param n:
    :param scale:
    :param xy_shift: shifts the initial position of the object on the scene.
    :param n_smooth:
    :return:
    '''

    xy2    = ut.resample_n_dim(n, xy)
    xy2    = scale * xy2 + xy_shift
    xy2    = np.vstack((xy2, xy2[0, :]))

    diff_xyz = np.diff(xy2[:, 0:2], axis=0)
    odom = np.sqrt(diff_xyz[:, 0] ** 2 + diff_xyz[:, 1] ** 2 )
    odom = np.cumsum(odom)
    odom = np.hstack(([0], odom))
    xy2  = ut.smooth_trajectory(odom, xy2, enforce_zero_vel=False)

    xy2[:, 0] = ut.smooth_simple(xy2[:, 0], n_smooth)
    xy2[:, 1] = ut.smooth_simple(xy2[:, 1], n_smooth)

    center = [0.5*(  np.min(xy2[:,0]) + np.max(xy2[:,0]) ), 0.5*(  np.min(xy2[:,1]) + np.max(xy2[:,1]) )]

    return {'xy':xy2, 'center':center}

def load_obstacles(prm_wipe, plot=False):

    n = 100

    if prm_wipe['obstacle_footprint'] == 'blob':

        file1 = '20200815_004029.txt'
        xy     = np.loadtxt(file1)
        obs1 = process_obstacle(xy, n=n, scale = [.5, .5], xy_shift = [0.2, 0.75], n_smooth=3)
        if plot:
            utp.figure_2D_list('obstacle1')
            plt.plot(obs1['xy'][:,0], obs1['xy'][:,1], marker='o')

        file2 = '20200815_001432.txt'  # blob
        xy     = np.loadtxt(file2)
        obs2   = process_obstacle(xy, n=n, scale = [.65, .65], xy_shift = [0.25, 1.05], n_smooth=33)

        if plot:
            utp.figure_2D_list('obstacle2')
            plt.plot(obs2['xy'][:,0], obs2['xy'][:,1], marker='o')

    if prm_wipe['obstacle_footprint'] == 'circle':

        t  = np.linspace(0,2*np.pi, n)
        xy = np.vstack((np.cos(t), np.sin(t))).T

        obs1 = process_obstacle(xy, n=n, scale=[.2217, .217],   xy_shift=[ 0.146,  0.20], n_smooth=3) # projector
        obs2 = process_obstacle(xy, n=n, scale=[.3, .3],   xy_shift=[ 0.146,  0.65], n_smooth=3) # laptop

    obstacle = [obs1, obs2]

    return obstacle



def pre_compute_all_primitives(prm_wipe, xy_path ):
    '''
    Before starting the task, already store all primitives that will be used.
    Note that these primitives will be adapted online by the obstacle avoidance method.
    :param wipe_type: 'long_stroke_with_circles', 'long_stroke', 'inside_out'
    :param xy_path:
    :param n_passes: You can specify the number of passes (the repetitions of the primitive). Otherwise
                     a pre-selected value will be used.
    :return:
    '''

    user_selected_table_type = prm_wipe['in_use']['table_type']
    user_selected_wipe_type  = prm_wipe['in_use']['wipe_type']
    wipe_motion_params       = prm_wipe[user_selected_table_type]['wipe_pattern'][user_selected_wipe_type]
    # wipe_type = prm_wipe['wipe_type']
    #

    xy_ = np.copy(xy_path)

    k_vec = range(wipe_motion_params['n_passes'])

    if user_selected_wipe_type in ['long_stroke_with_circles', 'long_stroke']:

        for k in k_vec:
            if k == 0:
                forward_pass = [xy_]
            else:
                shift_traj  = forward_pass[k-1][-1, :] - forward_pass[k-1][0, :]
                xy_next     = np.copy(forward_pass[k-1])
                xy_next     = xy_next + shift_traj
                forward_pass.append(xy_next)

    if user_selected_wipe_type in ['inside_out']:

        inside_out_scale = 0.75
        for k in k_vec:
            if k == 0:
                forward_pass = [xy_]
                dxy  = np.diff( xy_, axis=0 )
                odom = np.sqrt(dxy[:, 0] ** 2 + dxy[:, 1] ** 2)
                total_length_1 = np.cumsum(odom)[-1]
            else:
                start_next  = forward_pass[k-1][-1, :]
                xy_next     = np.copy(forward_pass[k-1])
                xy_next     = xy_next * inside_out_scale
                xy_next     = xy_next - xy_next[0, :] + start_next

                dxy  = np.diff( xy_next, axis=0 )
                odom = np.sqrt(dxy[:, 0] ** 2 + dxy[:, 1] ** 2)
                total_length_2 = np.cumsum(odom)[-1]

                # resample the trajectory to be proportional to the total length of the path
                n_resample = int( (total_length_2/total_length_1)*len(xy_[:,0]) )
                xy_next    = ut.resample_n_dim(n_resample, xy_next)

                forward_pass.append(xy_next)

    # reverse trajectories so the wiping goes back and forth and return a single set
    back_pass = forward_pass[::-1]
    for k in range(len(back_pass)):
        xy = np.flip( back_pass[k], axis=0)
        back_pass[k] = xy

    # plot the expected trace of paths here
    for k in range(len(forward_pass)):
        plt.plot( forward_pass[k][:,0], forward_pass[k][:,1], color=utp.color('k', .85), linewidth=2.25   )

    sequence_of_primitives = forward_pass + back_pass

    return sequence_of_primitives



def hard_coded_table_boundaries(table_type, n_traj=200, debug=False):

    # 'oblong_table or circular_table or square_table.'

    if table_type == 'square_table':

        x_dim = 0.8
        y_dim = 1.2
        x_ = [0.0, x_dim, x_dim, 0, 0]
        y_ = [0.0, 0    , y_dim, y_dim, 0]

        table_path_sparse = np.vstack((x_, y_)).T
        table_path = ut.resample(n_traj, table_path_sparse)

        xy_center = np.mean(table_path, axis=0)
        table_path = table_path-xy_center

        if debug:
            utp.figure_2D_list('table')
            plt.plot(table_path_sparse[:,0], table_path_sparse[:,1], marker='o', color='b')
            plt.plot(table_path[:, 0], table_path[:, 1], marker='o', color='r')


    if table_type == 'circular_table':
        rad = 0.5
        # create the round part
        t = np.linspace(0, 2*np.pi, n_traj)
        x1 = rad * np.cos(t)
        y1 = rad * np.sin(t)

        table_path = np.vstack((x1,y1)).T

    if table_type == 'oblong_table':

        rad = 0.8
        total_length = 3.2
        straight_line = total_length-2*rad

        # create the round part
        t = np.linspace(0, np.pi, 30)
        x1 = rad * np.cos(t)
        y1 = rad * np.sin(t)

        if debug:
            utp.figure_2D_list('table')
            plt.plot(x1,y1, '.', color='b')
            plt.plot(x1[0], y1[0], marker='o', markersize=10, color='b')
            utp.set_axes_equal_2D()

        # create the straight edge
        x0,y0 = np.linspace(x1[0], x1[0],30), -np.linspace(y1[0]+straight_line,y1[0],30)

        if debug:
            plt.plot(x0,y0, '.', color='r')
            plt.plot(x0[0], y0[0], marker='o', markersize=10, color='r')
            utp.set_axes_equal_2D()

        # another straight edge
        x2,y2 = np.linspace(x1[-1], x1[-1],30), -np.linspace(y1[0], y1[0]+straight_line,30)

        if debug:
            plt.plot(x2,y2, '.', color='g')
            plt.plot(x2[0], y2[0], marker='o', markersize=10, color='g')
            utp.set_axes_equal_2D()

        # the round part mirrored from the first part
        x3 = x1[::-1]
        y3 = -y1
        y3 = y3-y3[0]+y2[-1]

        if debug:
            plt.plot(x3,y3, '.', color='m')
            plt.plot(x3[0], y3[0], marker='o', markersize=10, color='m')
            utp.set_axes_equal_2D()

        x_ = np.concatenate((x0, x1, x2, x3))
        y_ = np.concatenate((y0, y1, y2, y3))
        table_path = np.vstack((x_, y_)).T

        table_path = ut.resample_based_on_traveled_distance(table_path,table_path, profile='none')
        table_path = ut.resample(n_traj, table_path)

        if debug:
            plt.plot(x_, y_, '.', color='y')
            for xy in table_path:
                plt.plot(xy[0], xy[1], marker='x')
                plt.pause(.051)

        # center table at zero
        xy_mean = np.mean(table_path, axis=0)
        table_path = table_path-xy_mean

    return table_path

def shift_table_position(v, table_center, xy_edge, xy_path_start):

    z_height = 0

    # add the height
    z_ = z_height* np.ones(xy_edge.shape[0])
    xyz_surface_boundary = np.hstack((xy_edge, z_[:,np.newaxis]  ))

    return xy_edge, xyz_surface_boundary



def move_obstacle(dxy, obs):

    obs['xy'] = obs['xy'] + dxy
    obs['center'] = [0.5 * (np.min(obs['xy'][:, 0]) + np.max(obs['xy'][:, 0])),
                     0.5 * (np.min(obs['xy'][:, 1]) + np.max(obs['xy'][:, 1]))]
    return obs

def refresh_path(avoid, keyboard, obs, xy_surface=[]):

    # ===========================================
    # change position of obstacle
    # ===========================================
    delta = 0.02
    # move the obstacle 1 on the screen. Use 'adsw' keys.
    dxy  = [0, 0]
    if keyboard == 'a':  dxy[0] = -delta
    if keyboard == 'd':  dxy[0] = +delta
    if keyboard == 's':  dxy[1] = -delta
    if keyboard == 'w':  dxy[1] = +delta
    obs[0] = move_obstacle(dxy, obs[0])

    dxy = [0, 0]
    if keyboard == 'j':  dxy[0] = -delta
    if keyboard == 'l':  dxy[0] = +delta
    if keyboard == 'k':  dxy[1] = -delta
    if keyboard == 'i':  dxy[1] = +delta
    obs[1] = move_obstacle(dxy, obs[1])


    # ===========================================
    # make collision free path
    # ===========================================
    xy = np.copy(avoid.xy_path_without_collision)

    # step 1, if obstacle crosses edge, then change the xy_surface
    if ut.notempty(xy_surface):
        xy_surface_adapted = np.copy(xy_surface[:, 0:2])
        xy = xy if ut.isempty(xy_surface) else avoid.adapt_robot_trajectory_to_boundary(xy, xy_surface_adapted[:, 0:2], debug=False)

    # step 2, avoid obstacles. Only works when the obstacles are entirely contained within the boundaries of the surface. Otherwise,
    # unexpected results may occur, such as the path moving outside the table
    for obstacle in obs:
        xy = avoid.get_path_around_single_obstacle(xy, obstacle)

    xy = avoid.smooth_and_resample(xy, smooth_n=3)

    avoid.xy_path = xy

    return obs