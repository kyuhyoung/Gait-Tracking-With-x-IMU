# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
# FEDERAL UNIVERSITY OF UBERLANDIA
# Faculty of Electrical Engineering
# Biomedical Engineering Lab
# ------------------------------------------------------------------------------
# Author: Italo Gustavo Sampaio Fernandes
# Contact: italogsfernandes@gmail.com
# Git: www.github.com/italogfernandes
# ------------------------------------------------------------------------------
# Description:
# ------------------------------------------------------------------------------
import sys
if sys.version_info.major == 3:
    from queue import Queue
    from PyQt5.QtGui import QBrush, QColor, QPen
elif sys.version_info.major == 2:
    from Queue import Queue
    from PyQt4.QtGui import QBrush, QColor, QPen

from pyqtgraph.Qt import QtCore
import pyqtgraph as pg
from pyqtgraph.ptime import time
from numpy import clip
# ------------------------------------------------------------------------------
from PyQtGraphHandler import PyQtGraphHandler, PyQtGraphSeries
# ------------------------------------------------------------------------------


class IMUGraphHandler(PyQtGraphHandler):
    def __init__(self, qnt_points=2000, parent=None, y_range=(-1, 1), app=None):
        """
        Handles a plotwidget (library: PyQtGraph) as a continuous plot.
        It allows a simple way to plot points as if they where scrolling in the screen.
        It has all the IMU series needed (Quaternions, Accelerometer, Gyroscope and Magnetrometer)
        NOTE: Magnetrometer not implemented
        See the code of the test function in this file for an example.
        :param qnt_points: The amount of points that your plot will have.
        :param parent: The parent of the plotwidget (QObject).
        :param y_range: A tuple telling the minimum and maximum value for the y-axis.
        :param app: The QAplication that holds this widget - Not essential, but if it's defined, it
        will try to force a update in the whole window every plot interaction.
        """
        PyQtGraphHandler.__init__(self, qnt_points, parent, y_range, app)

        self.plotWidget.removeItem(self.series.curve)

        self.quaternion_w = PyQtGraphSeries(self, pen='k', name="QW")
        self.quaternion_x = PyQtGraphSeries(self, pen='r', name="QX")
        self.quaternion_y = PyQtGraphSeries(self, pen='g', name="QY")
        self.quaternion_z = PyQtGraphSeries(self, pen='b', name="QZ")

        self.accelerometer_x = PyQtGraphSeries(self, pen='r', name="AX")
        self.accelerometer_y = PyQtGraphSeries(self, pen='g', name="AX")
        self.accelerometer_z = PyQtGraphSeries(self, pen='b', name="AX")

        self.gyroscope_x = PyQtGraphSeries(self, pen='r', name="GX")
        self.gyroscope_y = PyQtGraphSeries(self, pen='g', name="GX")
        self.gyroscope_z = PyQtGraphSeries(self, pen='b', name="GX")

    def set_curve_group_visible(self,which_group):
        q_visible = 'Q' in which_group.upper()
        a_visible = 'A' in which_group.upper()
        g_visible = 'G' in which_group.upper()

        self.quaternion_w.set_visible(q_visible)
        self.quaternion_x.set_visible(q_visible)
        self.quaternion_y.set_visible(q_visible)
        self.quaternion_z.set_visible(q_visible)

        self.accelerometer_x.set_visible(a_visible)
        self.accelerometer_y.set_visible(a_visible)
        self.accelerometer_z.set_visible(a_visible)

        self.gyroscope_x.set_visible(g_visible)
        self.gyroscope_y.set_visible(g_visible)
        self.gyroscope_z.set_visible(g_visible)

    def update(self):
        """
        This method is called automatically, you should not call it by yourself.

        It verifies how many points are in the buffer,
        then remove one by one, and add to the auxiliary vector.
        This auxiliary vector is set as the data source of the curve in the plot.
        """
        self.quaternion_w.update_values()
        self.quaternion_x.update_values()
        self.quaternion_y.update_values()
        self.quaternion_z.update_values()

        self.accelerometer_x.update_values()
        self.accelerometer_y.update_values()
        self.accelerometer_z.update_values()

        self.gyroscope_x.update_values()
        self.gyroscope_y.update_values()
        self.gyroscope_z.update_values()

        if self.show_fps:
            self.calculate_fps()
            self.plotWidget.setTitle('<font color="red">%0.2f fps</font>' % self.fps)

def test():
    import sys
    from PyQt4 import QtGui
    app = QtGui.QApplication(sys.argv)
    form = QtGui.QMainWindow()
    form.resize(800, 600)
    central_widget = QtGui.QWidget(form)
    vertical_layout = QtGui.QVBoxLayout(central_widget)

    plot_handler = IMUGraphHandler(qnt_points=800, parent=central_widget, y_range=[-2, 2])

    from datetime import datetime
    import numpy as np

    def generate_point():
        agr = datetime.now()
        y_value = agr.microsecond / 1000000.0
        accelx = np.sin(2*np.pi*y_value)            + 0.2*np.sin(20*2*np.pi*y_value)
        accely = np.sin(2*np.pi*y_value + 2*np.pi/3)+ 0.2*np.sin(20*2*np.pi*y_value + 2*np.pi/3)
        accelz = np.sin(2*np.pi*y_value - 2*np.pi/3)+ 0.2*np.sin(20*2*np.pi*y_value - 2*np.pi/3)
        plot_handler.accelerometer_x.buffer.put(accelx)
        plot_handler.accelerometer_y.buffer.put(accely)
        plot_handler.accelerometer_z.buffer.put(accelz)

    from ThreadHandler import InfiniteTimer
    timer = InfiniteTimer(0.005, generate_point)
    timer.start()

    plot_handler.timer.start(0)

    vertical_layout.addWidget(plot_handler.plotWidget)
    form.setCentralWidget(central_widget)
    form.show()
    app.exec_()

    timer.stop()
    plot_handler.timer.stop()

if __name__ == '__main__':
    test()