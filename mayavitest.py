import sys
# import vtk
# from PyQt5 import QtCore, QtGui
# from PyQt5 import Qt

# from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor

# class MainWindow(Qt.QMainWindow):

#     def __init__(self, parent = None):
#         Qt.QMainWindow.__init__(self, parent)

#         self.frame = Qt.QFrame()
#         self.vl = Qt.QVBoxLayout()
#         self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
#         self.vl.addWidget(self.vtkWidget)

#         self.ren = vtk.vtkRenderer()
#         self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
#         self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

#         # Create source
#         source = vtk.vtkSphereSource()
#         source.SetCenter(0, 0, 0)
#         source.SetRadius(5.0)

#         # Create a mapper
#         mapper = vtk.vtkPolyDataMapper()
#         mapper.SetInputConnection(source.GetOutputPort())

#         # Create an actor
#         actor = vtk.vtkActor()
#         actor.SetMapper(mapper)

#         self.ren.AddActor(actor)

#         self.ren.ResetCamera()

#         self.frame.setLayout(self.vl)
#         self.setCentralWidget(self.frame)

#         self.show()
#         self.iren.Initialize()
#         self.iren.Start()


# if __name__ == "__main__":
#     app = Qt.QApplication(sys.argv)
#     window = MainWindow()
#     sys.exit(app.exec_())


from PyQt5 import QtCore, QtGui
from PyQt5 import Qt
from PyQt5.QtWidgets import QVBoxLayout
import os


from traits.api import HasTraits, Range, Instance, \
                    on_trait_change
from traitsui.api import View, Item, HGroup
from tvtk.pyface.scene_editor import SceneEditor
from mayavi.tools.mlab_scene_model import \
                    MlabSceneModel
from mayavi.core.ui.mayavi_scene import MayaviScene


from numpy import linspace, pi, cos, sin
os.environ['ETS_TOOLKIT'] = 'pyqt5'


def curve(n_mer, n_long):
    phi = linspace(0, 2*pi, 2000)
    return [ cos(phi*n_mer) * (1 + 0.5*cos(n_long*phi)),
            sin(phi*n_mer) * (1 + 0.5*cos(n_long*phi)),
            0.5*sin(n_long*phi),
            sin(phi*n_mer)]

class Visualization(HasTraits):
    meridional = Range(1, 30,  6)
    transverse = Range(0, 30, 11)
    scene      = Instance(MlabSceneModel, ())

    def __init__(self):
        # Do not forget to call the parent's __init__
        HasTraits.__init__(self)
        x, y, z, t = curve(self.meridional, self.transverse)
        self.plot = self.scene.mlab.plot3d(x, y, z, t, colormap='Spectral')

    @on_trait_change('meridional,transverse')
    def update_plot(self):
        x, y, z, t = curve(self.meridional, self.transverse)
        self.plot.mlab_source.trait_set(x=x, y=y, z=z, scalars=t)


    # the layout of the dialog created
    view = View(Item('scene', editor=SceneEditor(scene_class=MayaviScene),
                    height=250, width=300, show_label=False),
                HGroup(
                        '_', 'meridional', 'transverse',
                    ),
                )
class MainWindow(Qt.QMainWindow):
    def __init__(self, parent=None):
        Qt.QMainWindow.__init__(self, parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        self.visualization = Visualization()

        # If you want to debug, beware that you need to remove the Qt
        # input hook.
        #QtCore.pyqtRemoveInputHook()
        #import pdb ; pdb.set_trace()
        #QtCore.pyqtRestoreInputHook()

        # The edit_traits call will generate the widget to embed.
        self.ui = self.visualization.edit_traits(parent=self,
                                                 kind='subpanel').control
        layout.addWidget(self.ui)
        self.ui.setParent(self)



if __name__ == "__main__":
    app = Qt.QApplication(sys.argv)
    window = MainWindow()
    window.setWindowTitle("hello")
    window.show()
    sys.exit(app.exec_())