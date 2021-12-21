from __future__ import print_function

import matplotlib.path as mpltPath
import matplotlib.pyplot as plt
import numpy as np

import utils as ut
import utils_plot as utp


class PATH_OBSTACLE_AVOIDANCE( ):

    def __init__(self, path=[]):
        '''
        You should not supply the path in principle. (it is kept here to maintain compatility).
        :param path:
        '''

        self.xy_path = []
        self.xy_path_without_collision = np.copy(path)
        pass

    def set_new_path(self, path):
        '''
        Use this to define a new path (free from collision).
        The effect is the same as creating the object from scratch.
        :param path:
        :return:
        '''
        self.xy_path = path
        self.xy_path_without_collision = np.copy(path)


    def figure_initialize(self, hxy, obstacles, boundary):

        self.plot_handle = {}

        # hxy = utp.figure_2D_list('xy_new', grid=False)
        self.plot_handle['hxy'] = hxy

        linewidth1 = 2

        utp.set_axes_equal_2D()
        utp.move_figure(hxy['fig'], 500, 300)

        if ut.notempty(boundary):
            l, = hxy['ax'].plot(boundary[:,0],
                                boundary[:,1],
                                color=utp.color('g',0.15),
                                linewidth=3,
                                )
            self.plot_handle['boundary'] = l



        l, = hxy['ax'].plot(self.xy_path[0,0],
                            self.xy_path[0,1],
                            color=utp.color('b',0),
                            marker='o',
                            markersize=7)
        self.plot_handle['xy_path_start'] = l

        l, = hxy['ax'].plot(self.xy_path[-1,0],
                            self.xy_path[-1,1],
                            color=utp.color('r',0),
                            marker='o',
                            markersize=7)
        self.plot_handle['xy_path_end'] = l


        l, = hxy['ax'].plot(self.xy_path[0,0],
                            self.xy_path[0,1],
                            color=utp.color('b',0),
                            marker='o',
                            markersize=15)
        self.plot_handle['xy_endeff'] = l



        # plot obstacles
        self.plot_handle['obstacles_center'] = []
        self.plot_handle['obstacles_xy']     = []

        linewidth2 = 4
        for k in range(len(obstacles)):

            l, = hxy['ax'].plot(obstacles[k]['center'][0],
                               obstacles[k]['center'][1],
                               color='k', marker='x', markersize=7)
            self.plot_handle['obstacles_center'].append(l)

            l, = hxy['ax'].plot(obstacles[k]['xy'][:,0],
                              obstacles[k]['xy'][:,1],
                               color=utp.color('r',0.1),
                               linewidth=linewidth2,
                                )

            self.plot_handle['obstacles_xy'].append(l)
            
        # plot path solution
        l, = hxy['ax'].plot(self.xy_path[:,0],
                            self.xy_path[:,1],
                            color=utp.color('k', 0),
                            linewidth=linewidth1,
                            )
        self.plot_handle['xy_path'] = l


        self.plot_handle['hxy']['fig'].canvas.draw()


    def figure_update(self, obs):

        self.plot_handle['xy_path'].set_xdata(self.xy_path[ :,0])
        self.plot_handle['xy_path'].set_ydata(self.xy_path[ :,1])

        self.plot_handle['xy_path_start'].set_xdata(self.xy_path[0,0])
        self.plot_handle['xy_path_start'].set_ydata(self.xy_path[0,1])

        self.plot_handle['xy_path_end'].set_xdata(self.xy_path[-1,0])
        self.plot_handle['xy_path_end'].set_ydata(self.xy_path[-1,1])


        for k in range(len(obs)):
            self.plot_handle['obstacles_xy'][k].set_xdata(obs[k]['xy'][:,0])
            self.plot_handle['obstacles_xy'][k].set_ydata(obs[k]['xy'][:,1])
            self.plot_handle['obstacles_center'][k].set_xdata(obs[k]['center'][0])
            self.plot_handle['obstacles_center'][k].set_ydata(obs[k]['center'][1])

        # self.plot_handle['hxy']['fig'].canvas.blit(  self.plot_handle['hxy']['fig'].bbox  )


    def contour_obstacle(self, xy, xy_obstacle, idx_inside_obstacle):

        n_traj_obstacle = xy_obstacle.shape[0]
        n_traj_inside_obstacle = len(idx_inside_obstacle)
        xy_new = np.copy(xy)

        # =====================================
        # analyze the two possibilities where the trajectory can go either to the left
        # or to the right of the obstacle
        # =====================================
        if idx_inside_obstacle != []:

            # point in obstacle where path hits it first time
            ind_start = np.argmin(np.sum((xy_obstacle - xy[idx_inside_obstacle[0], :]) ** 2, axis=1))
            # point in obstacle where path leaves the inner part of the obstacle
            ind_end = np.argmin(np.sum((xy_obstacle - xy[idx_inside_obstacle[-1], :]) ** 2, axis=1))

            new_index = list( range(ind_start, n_traj_obstacle))  + list(range(ind_start))

            # find the index where the trajectory leaves the inside of the obstacle
            # under the new, reindexed obstacle countour
            idx = []
            [idx.append(i) for i, x in enumerate(new_index == ind_end) if x]

            ind_forward = new_index[0:idx[0]]
            ind_backward = new_index[-1:idx[0]:-1]

            # this is important!!
            if ind_forward  == []: return xy_new
            if ind_backward == []: return xy_new

            # pick the one that has the shortest length
            if len(ind_forward) < len(ind_backward):
                path_obstacle = xy_obstacle[ind_forward, :]
            else:
                path_obstacle = xy_obstacle[ind_backward, :]

            path_obstacle_r = ut.resample(n_traj_inside_obstacle, path_obstacle)

            xy_new[idx_inside_obstacle, :] = path_obstacle_r

        return xy_new

    def get_path_around_single_obstacle(self, xy, obstacles):

        # compute the points from the path that are inside the obstacle
        path = mpltPath.Path(obstacles['xy'])
        inside2 = path.contains_points(xy)
        idx_inside_obstacle = []
        [idx_inside_obstacle.append(i) for i, x in enumerate(inside2) if x]

        # ========================================
        # break in segments of indices that fall inside obstacles.
        # If the path only crosses the obstable once (get in and get out)
        # once, there is only one segment.
        # ========================================
        seg = []
        g   = []
        for id in idx_inside_obstacle:
            if g == []:
                g.append(id)
                continue
            if id - g[-1] != 1:
                pass
                seg.append(g)
                g = []
                g.append(id)  # one block detected
            else:
                g.append(id)
        seg.append(g)

        # ========================================
        # for each segment, find the countour along the obstacle
        # ========================================
        xy_new = np.copy(xy)
        for k in range(len(seg)):
            xy_new = self.contour_obstacle(xy_new, obstacles['xy'], seg[k])

        return xy_new

    
    def adapt_robot_trajectory_to_boundary(self, xy_robot, edge_of_table, debug=False):

        debug = False

        path1_new = np.copy(xy_robot)

        # find the segment of the path2
        path1_in, path1_out = ut.intersecting_2D_segments(xy_robot, edge_of_table)

        if ut.notempty(path1_out)  and debug:
            plt.plot(edge_of_table[:,0], edge_of_table[:,1], color=utp.color('magenta',0))
            plt.plot(path1_in['xy'][:, 0], path1_in['xy'][:, 1], marker='o',
                     markersize=5, linewidth=.5, color=utp.color('r',0))
            plt.plot(path1_out['xy'][:, 0], path1_out['xy'][:, 1], marker='o',
                     markersize=6, linewidth=1.5, color=utp.color('b',0))


        if ut.notempty(path1_out):

            path2_in, path2_out = ut.intersecting_2D_segments(edge_of_table, xy_robot)

            if debug:
                plt.plot(path2_in['xy'][:, 0], path2_in['xy'][:, 1],
                         marker='o',
                         markersize=7, linewidth=0.5, color=utp.color('m', 0))
                plt.plot(path2_out['xy'][:, 0], path2_out['xy'][:, 1],
                         marker='o',
                         markersize=7, linewidth=1.5, color=utp.color('k', .8))

            # replace path1_out (robot trajectory) with path2_in (edge of table)


            # =====================================
            # analyze the two possibilities where the edge correction can go either to the left
            # or to the right of the obstacle. Pick the side that falls inside the edge closed shape.
            # =====================================
            xy = xy_robot[:]
            xy_obstacle = edge_of_table[:]
            n_traj_obstacle = xy_obstacle.shape[0]

            # point in obstacle where edge of surface hits it first time
            ind_start = np.argmin(np.sum((xy_obstacle - xy[path1_out['idx'][0], :] ) ** 2, axis=1))
            # point in obstacle where path leaves the inner part of the obstacle
            ind_end = np.argmin(np.sum((xy_obstacle - xy[path1_out['idx'][-1], :]) ** 2, axis=1))

            new_index = list( range(ind_start, n_traj_obstacle)  ) + list( range(ind_start))

            # find the index where the trajectory leaves the inside of the obstacle
            # under the new, reindexed obstacle countour
            idx = []
            [idx.append(i) for i, x in enumerate(new_index == ind_end) if x]

            ind_forward  = new_index[0:idx[0]]
            ind_backward = new_index[-1:idx[0]:-1]

            # not sure if this is necessary.
            if ind_forward == []:
                return path1_new
            if ind_backward == []:
                return path1_new

            # criterion to know if you should pick ind_backward or ind_forward is if they have the same
            # elements as in path2_in['idx'] because the latter contains the points inside the countour
            # of the table.

            seg_optionA = ut.resample(100, xy_obstacle[ind_backward, :])
            seg_optionB = ut.resample(100, xy_obstacle[ind_forward, :])
            seg_ref     = ut.resample(100, path2_in['xy'])

            distA = np.sum(np.sqrt( np.sum(  (seg_optionA-seg_ref)**2, axis=1  )))
            distB = np.sum(np.sqrt(np.sum((seg_optionB - seg_ref) ** 2, axis=1)))

            if distA < distB:
                segment_obstacle = xy_obstacle[ind_backward, :]
            else:
                segment_obstacle = xy_obstacle[ind_forward, :]


            segment_obstacle_r = ut.resample(len(path1_out['idx']), segment_obstacle)
            path1_new[path1_out['idx'], :] = segment_obstacle_r  # plt.plot(path_obstacle_r[0,:], path_obstacle_r[1,:], color='b', marker='.')

        return path1_new



    def smooth_and_resample(self, xy, smooth_n=[]):

        if smooth_n != []:
            xy[:, 0] = ut.smooth_simple(xy[:, 0], smooth_n)
            xy[:, 1] = ut.smooth_simple(xy[:, 1], smooth_n)

        # resample trajectory to have constant stepsize again so that velocity is roughly constant
        # all along the trajectory, but this does not guarantee zero velocity at start/end
        diff_xyz = np.diff(xy[:, 0:2], axis=0)
        odom = np.sqrt(diff_xyz[:, 0] ** 2 + diff_xyz[:, 1] ** 2)
        odom = np.cumsum(odom)
        odom = np.hstack(([0], odom))
        xy   = ut.smooth_trajectory(odom, xy, enforce_zero_vel=False)

        return xy

def main():
    pass

if __name__ == "__main__":

    main()


















