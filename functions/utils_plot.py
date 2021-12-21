
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
#import utils as ut
#from mpl_toolkits.mplot3d import Axes3D

# from screeninfo import get_monitors

#
#
# def set_font_size(size = 12):
#     font = {'family': 'normal', 'size': size}
#     plt.rc('font', **font)
#
#
# class Monitor_Setup:
#
#     def __init__(self):
#
#         self.configuration = []
#
#         for m in get_monitors():
#             print(str(m))
#
#     def set_position_by_name(self, h):
#
#         if h['name'] is 'asdf':
#             x_pos = 1000
#             y_pos = 500
#
#         utp.move_figure(h['fig'], x=x_pos, y=y_pos)
#
#
# def help_label():
#
#     print('Example:\n')
#     print('line_up,   = plt.plot([1, 2, 3], label=\'Line 2\')')
#     print('line_down, = plt.plot([3, 2, 1], label=\'Line 1\') ')
#     print('plt.legend(handles=[line_up, line_down]) ')
#
def color(colorname, intensity=0.6):

    if colorname is 'r':
        rgbval = [1, intensity, intensity]

    if colorname is 'red':
        rgbval = [1, intensity, intensity]

    if colorname is 'g':
        rgbval = [intensity, 1, intensity]

    if colorname is 'green':
        rgbval = [intensity, 1, intensity]

    if colorname is 'b':
        rgbval = [intensity, intensity, 1]

    if colorname is 'blue':
        rgbval = [intensity, intensity, 1]


    if colorname is 'k':
        rgbval = [intensity, intensity, intensity]

    if colorname is 'gray':
        rgbval = [intensity, intensity, intensity]

    if colorname is 'y':
        rgbval = [0.8, 0.8, intensity]
    if colorname is 'yellow':
        rgbval = [0.8, 0.8, intensity]

    if colorname is 'magenta' or colorname is 'm':
        rgbval = [1, intensity, 1]

    return rgbval
#
# class struct_plot_handle():
#
#     def __init__(self, name):
#         self.name = name
#         self.ax = []
#         self.fig = []
#
#
# class struct_fake():
#     def __init__(self):
#         pass
#
#
def hold():
    plt.show(block=True)
#
def move_figure(f, x=250, y=0):
    """
    :param f: is fig
    :param x:
    :param y:
    :return:
    """
    """Move figure's upper left corner to pixel (x, y)"""

    backend = matplotlib.get_backend()
    if backend == 'TkAgg':
        f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
    elif backend == 'WXAgg':
        f.canvas.manager.window.SetPosition((x, y))
    else:
        # This works for QT and GTK
        # You can also use window.setGeometry
        f.canvas.manager.window.move(x, y)
    plt.show()
#
#
# def quick_figure(y):
#     plt.figure(ut.string_date_time()[-6:])
#     plt.plot(y)
#
# def size_figure(fig, x=6, y=6):
#     resize_figure(fig, x, y)
#
#
# def resize_figure(fig, x=6, y=6):
#     fig.set_size_inches(x, y)
#
#
# def figure_2D_listA(name, x=50, y=70, xsize=6, ysize=6):
#
#     hs = {}
#     hs['name'] = name
#
#     hs['fig'], hs['ax'] = plt.subplots()
#     plt.ion()
#     plt.pause(0.00001)
#     hs['fig'].canvas.draw()
#
#     move_figure(hs['fig'], x, y)
#
#     if xsize:
#         hs['fig'].set_size_inches(xsize, ysize)
#
#     hs['ax']  = hs['fig'].add_subplot(111)
#     plt.grid(color('k', .7))
#
#     return hs
#
#
def figure_2D_list(name, x=50, y=120, xsize=6, ysize=6, grid=True):

    hs = {}
    hs['name'] = name
    hs['fig']  = plt.figure(name)
    plt.ion()
    plt.pause(0.00001)
    hs['fig'].canvas.draw()

    move_figure(hs['fig'], x, y)

    if xsize:
        hs['fig'].set_size_inches(xsize, ysize)

    hs['ax']  = hs['fig'].add_subplot(111)

    if grid:
        plt.grid(color('k', .7))

    return hs
#
# def figure_2D_subplot_row(name, n_row=2,  x=1000, y=0, xsize=6, ysize=12):
#     '''
#     Create a plot with n_row of subplots
#     :param name:
#     :param n_row: default is 2
#     :param x: position on screen
#     :param y: position on screen
#     :param xsize:
#     :param ysize:
#     :return:
#     '''
#
#     hs = {}
#     hs['name'] = name
#     hs['fig']  = plt.figure(name)
#     plt.ion()
#     plt.show()
#     hs['fig'].canvas.draw()
#     move_figure(hs['fig'], x, y)
#
#     if xsize:
#         hs['fig'].set_size_inches(xsize, ysize)
#
#     for j in range(n_row):
#         name_key = 'ax%.1d' %(j+1)
#         hs[name_key] = hs['fig'].add_subplot(n_row, 1, j+1 )
#         hs[name_key].grid(color=[0.58, 0.58, 0.58], linestyle='--', linewidth=0.5)
#
#     set_subplot_active(hs['ax1'])
#
#     return hs
#
# def plot_recenter(ax, xyz_center, delta):
#
#     ax.set_xlim([xyz_center[0]-0.5*delta, xyz_center[0]+0.5*delta])
#     ax.set_ylim([xyz_center[1] - 0.5 * delta, xyz_center[1] + 0.5 * delta])
#     ax.set_zlim([xyz_center[2]-0.5*delta, xyz_center[2]+0.5*delta])
#
# def update_plot(plot_struc=False, aspect_equal=False):
#
#     if plot_struc is False:
#         ax = plt.gca()
#     else:
#         ax = plot_struc.ax
#
#     ax.relim()
#     ax.autoscale_view()
#     # ax.set_aspect('equal')
#
#     if aspect_equal is True:
#         set_axes_equal_3D(ax)
#
#     plt.show
#     plt.draw()
#     plt.pause(0.0001)
#
def set_axes_equal_2D():
    plt.axis('equal')
#
# def set_subplot_active(ax):
#     '''
#     make current axes active.
#     :param ax: ex.: fig['ax']
#     :return:
#     '''
#     plt.axes(ax)
#
# def plot_workspace_3d_list(name, xy_size=[]):
#     # proj3d.persp_transformation = orthogonal_proj
#
#     hs = {}
#     hs['fig'] = plt.figure(name)
#
#     if 0: #xy_size !=[]:
#         hs['fig'].set_size_inches(xy_size[0], xy_size[1])
#
#
#     hs['ax']  = plt.gca()
#
#     hs['ax'] = hs['fig'].add_subplot(111, projection='3d')
#     hs['ax'].w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     hs['ax'].w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     hs['ax'].w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#
#     plt.ion()
#     hs['fig'].show()
#
#     hs['ax'].set_xlabel('X (m)')
#     hs['ax'].set_ylabel('Y (m)')
#     hs['ax'].set_zlabel('Z (m)')
#
#     hs['ax'].dist = 8
#     hs['ax'].azim = -20
#     hs['ax'].elev = 75
#
#     hs['name'] = name
#
#     # set_axes_equal_3D(hs['ax'])
#
#     return hs
#
# def plot_workspace_3d(name):
#     # proj3d.persp_transformation = orthogonal_proj
#
#     hs = struct_fake()
#     hs = struct_plot_handle(name)
#     hs.fig = plt.figure(name)
#     hs.ax  = plt.gca()
#
#     hs.ax = hs.fig.add_subplot(111, projection='3d')
#     hs.ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     hs.ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#     hs.ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
#
#     plt.ion()
#     hs.fig.show()
#
#     hs.ax.set_xlabel('X (m)')
#     hs.ax.set_ylabel('Y (m)')
#     hs.ax.set_zlabel('Z (m)')
#
#     hs.ax.dist = 8
#     hs.ax.azim = -20
#     hs.ax.elev = 75
#
#     set_axes_equal_3D(hs.ax)
#
#     return hs
#
# def set_axes_equal_3D(ax):
#     '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
#     cubes as cubes, etc..  This is one possible solution to Matplotlib's
#     ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
#
#     Input
#       ax: a matplotlib axis, e.g., as output from plt.gca().
#     '''
#
#     x_limits = ax.get_xlim3d()
#     y_limits = ax.get_ylim3d()
#     z_limits = ax.get_zlim3d()
#
#     x_range = abs(x_limits[1] - x_limits[0])
#     x_middle = np.mean(x_limits)
#     y_range = abs(y_limits[1] - y_limits[0])
#     y_middle = np.mean(y_limits)
#     z_range = abs(z_limits[1] - z_limits[0])
#     z_middle = np.mean(z_limits)
#
#     # The plot bounding box is a sphere in the sense of the infinity
#     # norm, hence I call half the max range the plot radius.
#     plot_radius = 0.5*max([x_range, y_range, z_range])
#
#     ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
#     ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
#     ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
#
# def figure_2D_list_DELETABLE(name, x=50, y=50, xsize=6, ysize=6):
#     hs = {}
#     hs['name'] = name
#     hs['fig'] = plt.figure(name)
#     hs['ax'] = plt.gca()
#     plt.ion()
#     # plt.show()
#     hs['fig'].canvas.draw()
#     plt.grid(color('k', .7))
#
#     move_figure(hs['fig'], x, y)
#
#     if xsize:
#         hs['fig'].set_size_inches(xsize, ysize)
#
#     return hs
#
#
#
#
#
#
#
#
#
# def figure_2D(name, x=500, y=500, xsize=6, ysize=6):
#
#     hs = struct_fake()
#     hs = struct_plot_handle(name)
#     hs.fig = plt.figure(name)
#     hs.ax  = plt.gca()
#     plt.ion()
#     plt.show()
#     hs.fig.canvas.draw()
#
#     move_figure(hs.fig, x, y)
#
#     if xsize:
#         hs.fig.set_size_inches(xsize, ysize)
#
#     return hs
#
#
