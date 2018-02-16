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
from IMUGraphHandler import IMUGraphHandler
from QtArduinoPlotter import QtArduinoPlotter
# ------------------------------------------------------------------------------


class IMUArduinoPlotter(QtArduinoPlotter):
    def __init__(self, parent, app=None, label=None):
        QtArduinoPlotter.__init__(self, parent, app, label)
        self.plotHandler.set_curve_group_visible('G')

    def get_buffers_status(self, separator):
        """
        Returns a string like:
            Serial:    4/1024 - Acq:    1/1024 - Plot:  30/1024
        :param separator: Separates the strings, example ' - ', ' | ', '\n'
        :return: A string containing the status of all the buffers involved in the acquisition and plotting.
        """
        return self.arduinoHandler.get_buffers_status(separator) + separator + \
               self.plotHandler.accelerometer_x.get_buffers_status()

    def _init_plotHandler(self, parent, app):
        """
        Only initializes the plotHandler object. It is set as a method to allow override.
        """
        self.plotHandler = IMUGraphHandler(qnt_points=800, parent=parent, y_range=(-2, 2),
                                          app=app)

    def consumer_function(self):
        if self.arduinoHandler.data_waiting:
            packet_acquired = self.arduinoHandler.buffer_acquisition.get()
            self.plotHandler.quaternion_w.buffer.put(packet_acquired[0] / 16384.0)
            self.plotHandler.quaternion_x.buffer.put(packet_acquired[1] / 16384.0)
            self.plotHandler.quaternion_y.buffer.put(packet_acquired[2] / 16384.0)
            self.plotHandler.quaternion_z.buffer.put(packet_acquired[3] / 16384.0)
            self.plotHandler.accelerometer_x.buffer.put(packet_acquired[4] / 8192.0) # +4 ... work around...
            self.plotHandler.accelerometer_y.buffer.put(packet_acquired[5] / 8192.0) # i only want to plot in
            self.plotHandler.accelerometer_z.buffer.put(packet_acquired[6] / 8192.0) # separete plots
            self.plotHandler.gyroscope_x.buffer.put(packet_acquired[7] / 131.0)
            self.plotHandler.gyroscope_y.buffer.put(packet_acquired[8] / 131.0)
            self.plotHandler.gyroscope_z.buffer.put(packet_acquired[9] / 131.0)


def test():
    import sys
    from PyQt4 import QtGui
    app = QtGui.QApplication(sys.argv)
    form = QtGui.QMainWindow()
    form.resize(800, 600)
    central_widget = QtGui.QWidget(form)
    vertical_layout = QtGui.QVBoxLayout(central_widget)

    harry_plotter = IMUArduinoPlotter(parent=central_widget)#, app=app)
    harry_plotter.start()

    vertical_layout.addWidget(harry_plotter.plotHandler.plotWidget)
    form.setCentralWidget(central_widget)
    form.show()
    app.exec_()
    harry_plotter.stop()

if __name__ == '__main__':
    test()