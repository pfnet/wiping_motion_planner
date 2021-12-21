
import sys
sys.path.append('./functions/')

import KeyboardInput
import utils as ut
import utils_plot as utp
import time
import numpy as np
import PATH_OBSTACLE as PO
import matplotlib.pyplot as plt
import argparse
import aux_planner as auxpln


def main(prm_wipe,
         interface_type='ide',
         ):
    '''
    Examples
        :param interface_type: if using an IDE, set it to 'ide', otherwise 'terminal'
    '''


    v, xyz_pathstart = [], [0,0,0]

    keyboard_input = KeyboardInput.KeyboardInput(interface_type) if True else []

    # initialize the trajectory relative to the scene
    xy_path = np.array(auxpln.create_initial_cleaning_path(prm_wipe)).T
    xy_path = xy_path + xy_path[0, :]

    avoid = PO.PATH_OBSTACLE_AVOIDANCE(xy_path)
    obs   = auxpln.load_obstacles(prm_wipe, plot = False)

    xy_raw_edge_coord = auxpln.hard_coded_table_boundaries(prm_wipe['in_use']['table_type'], n_traj=200, debug=0)

    table_center = prm_wipe[prm_wipe['in_use']['table_type']]['objects'][0]
    xy_table2pathstart, xyz_table_in_scene = auxpln.shift_table_position(v, table_center, xy_raw_edge_coord, xyz_pathstart)

    hxy = utp.figure_2D_list('xy_new', grid=False)
    hxy['ax'].xaxis.set_visible(False)
    hxy['ax'].yaxis.set_visible(False)
    hxy['ax'].set_frame_on(False)
    hxy['fig'].patch.set_alpha(0.)
    hxy['fig'].tight_layout()
    hxy['fig'].canvas.draw()

    sequence_primitives = auxpln.pre_compute_all_primitives(prm_wipe, xy_path)

    auxpln.refresh_path(avoid, [], obs, xy_table2pathstart)

    avoid.figure_initialize(hxy, obs, xy_table2pathstart)
    plt.ylim(-0.01, 0.5)
    plt.xlim(-.9, 1.1)

    loop_freq, k_ctr = 60, 0 # Hz

    # =========================================================
    # tune parameters specifically for the task
    # =========================================================
    k_ctr = 0

    while 1: # use this to run the code indefinitely

        if k_ctr == len(sequence_primitives):
            k_ctr = 0 # keep an infinite loop of primitives by zeroing the index

        # retrieve the next primitive in the sequence
        avoid.xy_path_without_collision = sequence_primitives[k_ctr]

        time_  = time.time()
        k_ctr += 1
        n_path = len(avoid.xy_path_without_collision[:, 0])
        for tt in range(   n_path   ) :

            # ==========================================================
            # High frequency. You can only interact with this part via keyboard
            # ==========================================================
            time0_   = time.time()

            kval   = keyboard_input.get_val(v=v, kill_at_ESC=True) if keyboard_input else []

            obs = auxpln.refresh_path(avoid, kval, obs, xy_surface=xy_table2pathstart)

            if np.mod(tt, 4) == 0:
                avoid.figure_update(obs)

            if np.mod(tt, 2) == 0:
                avoid.plot_handle['xy_endeff'].set_xdata(avoid.xy_path[tt, 0])
                avoid.plot_handle['xy_endeff'].set_ydata(avoid.xy_path[tt, 1])
                #avoid.plot_handle['hxy']['fig'].canvas.blit(avoid.plot_handle['hxy']['fig'].bbox)

                #avoid.plot_handle['hxy']['fig'].canvas.blit(avoid.plot_handle['hxy']['fig'].bbox)

            plt.pause(0.00001)

            freq_raw = 1.0/(time.time()-time0_)

            while ( (time.time()-time0_) < (1.0/loop_freq) ) :
                pass # hold the loop to guarantee constant frequency

            # print freq once in a while
            if  (time.time()-time_) > 1  :
                str_ = '%.1f (Hz)' %(1.0/(time.time()-time0_))
                str_ = str_ + "/%.1f" %(freq_raw)
                print( str_ )
                time_ = time.time()



if __name__ == "__main__":

    parameters_wipe = ut.load_yaml('wipe_params.yaml')
    parameters_wipe['in_use'] = {}
    parameters_wipe['in_use']['wipe_type']  = 'inside_out'   # 'long_stroke_with_circles' or 'long_stroke' or 'inside_out'
    parameters_wipe['in_use']['table_type'] = 'circular_table'   # 'oblong_table' or 'circular_table' or 'square_table'


    print(    parameters_wipe['in_use']['table_type']  )


    main(parameters_wipe,
         interface_type = 'terminal'
         )








