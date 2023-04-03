
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 30 16:26:41 2020

@author: Aamir
"""

import numpy as np
import math
import pyvista as pv
import PVGeo as pvg
from pyvistaqt import BackgroundPlotter
#from PVGeo.filters import RotatePoints
#from PVGeo.filters import RotationTool

# Create a unit vector


def unit_vector(vec):
    unit = vec / np.linalg.norm(vec)
    return unit


points = np.array([[0, 0, 0],
                   [0, 0.5, 0.5],
                   [0.5, 0, 0.5],
                   [0.5, 0.5, 0]])


# Find the angle between two unit vector
def angle(vec1, vec2):
    v1_unit = unit_vector(vec1)
    v2_unit = unit_vector(vec2)
    theta = (np.arccos(np.clip(np.dot(v1_unit, v2_unit), -1.0, 1.0))) * (180/np.pi)
    return theta


# Create an accelerometer class
class Accelerometer():
    def __init__(self, p, N):

        # Creates an accelerometer
        self.box = pv.Box()
        self.box.translate([1, 1, 1])
        self.box.points /= 2

        p.add_mesh(self.box, opacity=0.3, show_edges=True, color="#8c8c8c")

        # Create x,y,z axis for the accelermoter box
        ray_x = pv.Line([0, 0, 0], [1, 0, 0])
        p.add_mesh(ray_x, color="r", line_width=3)
        ray_y = pv.Line([0, 0, 0], [0, 1, 0])
        p.add_mesh(ray_y, color="g", line_width=3)
        ray_z = pv.Line([0, 0, 0], [0, 0, 1])
        p.add_mesh(ray_z, color="b", line_width=3)

        self.accelerometer = [self.box, ray_x, ray_y, ray_z]
        self.N = int(N*4)

    # Create a function to translate the items of an acceleromter
    def translation(self, point):
        _new = point - self.box.center_of_mass() + [0.5, 0.5, 0.5]
        for item in self.accelerometer:
            item.translate(_new)

        for i in range(4):
            p.sphere_widgets[self.N + i].SetCenter(
                _new + np.asarray(p.sphere_widgets[self.N + i].GetCenter()))

   # A callback function to be called on clicking any of the four sphere widgets

    def callback(self, point, *args):
        print(point, *args)
        # 3D translation in space (on clicking the black widget)
        # if i == 0:

        _new = point - self.box.center_of_mass() + [0.5, 0.5, 0.5]

        for item in self.accelerometer:
            item.translate(_new)

        for k in range(3):
            p.sphere_widgets[self.N+1+k].SetCenter(
                _new + np.asarray(p.sphere_widgets[self.N+1+k].GetCenter()))

        # # If red widget is clicked (rotation of the accelerometer block about x axis)
        # elif i == 1:
        # # Rotation
        #     _new = self.box.center_of_mass() - [0.5,0.5,0.5]

        #     _vec1 = np.asarray(point-_new)
        #     _vec2 = np.asarray(points[1,:])
        #     theta = angle(_vec2, _vec1)
        #     #print(theta)

        #     for item in self.accelerometer:
        #         origin = item.bounds[0::2]
        #         pvg.filters.RotatePoints(origin, theta).apply(item)

        #         for k in range(3):

        #             p.sphere_widgets[self.N+1+k].SetCenter( points[1+k,:])

        # # If green widget is clicked (rotation of the accelerometer block about y axis)
        # elif i == 2:
        #     # Rotation
        #     _new = self.box.center_of_mass() - [0.5,0.5,0.5]

        #     _vec1 = np.asarray(point-_new)
        #     _vec2 = np.asarray(points[2,:])

        #     theta = angle(_vec1, _vec2)
        #     print(theta)

        #     for item in self.accelerometer:
        #         origin=item.bounds[0::2]
        #         pvg.filters.RotatePoints(origin, theta).apply(item)

        #     for k in range(3):
        #         p.sphere_widgets[self.N+1+k].SetCenter( points[1+k,:])

        # # If blue widget is clicked (rotation of the accelermeter block about z axis)
        # elif i == 3:
        #     # Rotation
        #     _new = self.box.center_of_mass() - [0.5,0.5,0.5]

        #     _vec1 = np.asarray(point-_new)
        #     _vec2 = np.asarray(points[3,:])

        #     theta = angle(_vec1, _vec2)
        #     print(theta)

        #     for item in self.accelerometer:
        #         origin=item.bounds[0::2]
        #         pvg.filters.RotatePoints(origin, theta).apply(item)

        #     for k in range(3):
        #         p.sphere_widgets[self.N+1+k].SetCenter(_new + points[1+k,:])


if __name__ == '__main__':

    # the option Background plotter helps to manipulate in the real time
    p = BackgroundPlotter()
    p.background_color = "#D4D4D4"
    p.show()

    # Changing the value of range, changes the number of accelerometer
    # for i in range(3):
    _gg = Accelerometer(p, 0)
    p.add_sphere_widget(_gg.callback, center=points, color=[
                        "k", "r", "g", "b"], radius=0.05)
    # _gg.translation(np.random.random(3)*10)
