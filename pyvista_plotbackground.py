from threading import Thread
import time
import numpy as np
import pyvista as pv
import pyvistaqt as pvqt
from pyvista import examples


globe = examples.load_globe()
globe.point_data['scalars'] = np.random.rand(globe.n_points)
globe.set_active_scalars('scalars')


plotter = pvqt.BackgroundPlotter()
plotter.add_mesh(globe, lighting=False, show_edges=True, texture=True, scalars='scalars')
plotter.view_isometric()
def shrink():
    for i in range(50):
        globe.points *= 0.95
        # Update scalars
        globe.point_data['scalars'] = np.random.rand(globe.n_points)
        time.sleep(0.5)

thread = Thread(target=shrink)
thread.start()