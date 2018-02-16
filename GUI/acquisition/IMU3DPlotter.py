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
import numpy as np
# ------------------------------------------------------------------------------


class IMU3DPlotter(QtArduinoPlotter):
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
        self.plotHandler = IMUGraphHandler(qnt_points=800, parent=parent, y_range=(-250, 250),
                                          app=app)

    def consumer_function(self):
        if self.arduinoHandler.data_waiting:
            packet_acquired = np.array(self.arduinoHandler.buffer_acquisition.get(), dtype=np.float64)
            packet_acquired[0:4] = packet_acquired[0:4] / 16384.0
            packet_acquired[4:7] = packet_acquired[4:7] / 8192.0  # 8192 for dmp and 16384 for raw data (deg/s)
            packet_acquired[7:10] = packet_acquired[7:10] / 8.2  # 8.2 for dmp and 131.0 for raw data (deg/s)
            # TODO: Ao inves de jogar no plot handler, processar, e jogar no 3d graph handler

            Fs = 200
            samplePeriod = 1.0 / Fs

            gyrX_last = gyrX
            gyrY_last = gyrY
            gyrZ_last = gyrZ
            accX_last = accX
            accY_last = accY
            accZ_last = accZ
            quat_last = quat

            gyrX = packet_acquired[7]
            gyrY = packet_acquired[8]
            gyrZ = packet_acquired[9]
            accX = packet_acquired[4]
            accY = packet_acquired[5]
            accZ = packet_acquired[6]
            quat = packet_acquired[0:4]

            # -------------------------------------------------------------------------
            # Detect stationary periods
            # TODO: improve filtering methods
            # Compute accelerometer magnitude
            acc_mag = np.sqrt(accX ** 2 + accY ** 2 + accZ ** 2)
            # HP filter accelerometer data using moving average
            acc_magFilt = abs(acc_mag) - 1

            # Compute absolute value
            acc_magFilt = abs(acc_magFilt)

            # LP filter accelerometer data
            acc_magFilt = acc_magFilt

            # TODO: build a calibration method
            # Threshold detection
            # stationaty_start_time = acc_magFilt[:(tempo_parado) * Fs]
            # statistical_stationary_threshold = np.mean(stationaty_start_time) + 2 * np.std(stationaty_start_time)
            stationary_threshold = 0.048
            stationary = acc_magFilt < stationary_threshold

            # -------------------------------------------------------------------------
            # TODO: Plot data raw sensor data and stationary periods
            # first subplot: gyrX, gyrY, gyrZ
            # second subplt: accX, accY, accZ, acc_magFilt, stationary

            # -------------------------------------------------------------------------
            # Compute orientation
            # NOTE: No need to compute orientation, since the acquirer uC is computing it
            #       maybe, it's a good idea to compute here, changing the beta parameter
            #       in the stationary and not stationary periods. It needs to be figured out.

            # -------------------------------------------------------------------------
            # Compute translational accelerations
            import quaternion_toolbox
            # Rotate body accelerations to Earth frame
            a = np.array([accX, accY, accZ]).T
            acc = quaternion_toolbox.rotate(a, quaternion_toolbox.conjugate(quat))

            # Convert acceleration measurements to m/s/s
            acc = acc * 9.81

            # TODO: Plot translational accelerations
            # Plot lines: acc[:, 0], acc[:, 1], acc[:, 2]

            # -------------------------------------------------------------------------
            # Compute translational velocities

            acc[:, 2] = acc[:, 2] - 9.81  # Remove gravity from measurements

            # Integrate acceleration to yield velocity
            vel = np.zeros(np.shape(acc))
            vel = vel_last + acc * samplePeriod
            if stationary[t]:
                vel = np.zeros((3))  # force zero velocity when foot stationary

            # Compute integral drift during non-stationary periods

            velDrift = np.zeros(np.shape(vel))

            stationaryStart_index = None
            stationaryEnd_index = None
            driftRate = vel[stationaryEnd] / (stationaryEnd_index - stationaryStart_index)
            enum = np.arange(0, (stationaryEnd_index - stationaryStart_index))
            enum_t = enum.reshape((1, len(enum)))
            driftRate_t = driftRate.reshape((1, len(driftRate)))

            drift = enum_t.T * driftRate_t

            velDrift[stationaryStart[i]:stationaryEnd[i], :] = drift

            # Remove integral drift
            vel = vel - velDrift

            # Plot translational velocity
            plt.figure(figsize=(20, 10))
            plt.suptitle('Velocity', fontsize=14)
            plt.grid()
            plt.plot(time, vel[:, 0], 'r')
            plt.plot(time, vel[:, 1], 'g')
            plt.plot(time, vel[:, 2], 'b')
            plt.title('Velocity')
            plt.xlabel('Time (s)')
            plt.ylabel('Velocity (m/s)')
            plt.legend(('X', 'Y', 'Z'))

            # -------------------------------------------------------------------------
            # Compute translational position

            # Integrate velocity to yield position
            pos = np.zeros(np.shape(vel))
            for t in range(1, len(pos)):
                pos[t, :] = pos[t - 1, :] + vel[t, :] * samplePeriod  # integrate velocity to yield position

            # Plot translational position
            plt.figure(figsize=(20, 10))
            plt.suptitle('Position', fontsize=14)
            plt.grid()
            plt.plot(time, pos[:, 0], 'r')
            plt.plot(time, pos[:, 1], 'g')
            plt.plot(time, pos[:, 2], 'b')
            plt.title('Position')
            plt.xlabel('Time (s)')
            plt.ylabel('Position (m)')
            plt.legend(('X', 'Y', 'Z'))

            print('Erro em Z: %.4f' % abs(pos[-1, 2]))
            # -------------------------------------------------------------------------
            #  Plot 3D foot trajectory

            # # Remove stationary periods from data to plot
            # posPlot = pos(find(~stationary), :)
            # quatPlot = quat(find(~stationary), :)
            posPlot = pos
            quatPlot = quat

            # Extend final sample to delay end of animation
            extraTime = 20
            onesVector = np.ones((extraTime * Fs, 1))
            # TODO: usar pading
            # np.pad()
            # posPlot = np.append(arr = posPlot, values = onesVector * posPlot[-1, :])
            # quatPlot = np.append(arr = quatPlot, values = onesVector * quatPlot[-1, :])

            # -------------------------------------------------------------------------
            # Create 6 DOF animation
            # TODO: improve it

            import numpy as np
            import matplotlib.pyplot as plt
            import mpl_toolkits.mplot3d.axes3d as p3
            import matplotlib.animation as animation

            posPlot = posPlot.T

            #
            # Attaching 3D axis to the figure
            fig = plt.figure()
            ax = p3.Axes3D(fig)

            data_x = posPlot[0, 0:1500]
            data_y = posPlot[1, 0:1500]
            data_z = posPlot[2, 0:1500]
            # Creating fifty line objects.
            # NOTE: Can't pass empty arrays into 3d version of plot()
            line = ax.plot(data_x, data_y, data_z)
            line = line[0]

            # Setting the axes properties
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            ax.set_title('3D Animation')

            ax.set_xlim3d([-3.0, 3.0])
            ax.set_ylim3d([-3.0, 3.0])
            ax.set_zlim3d([-3.0, 3.0])

            #
            def update_lines(num):
                # NOTE: there is no .set_data() for 3 dim data...
                index = num * 10
                line.set_data(posPlot[0:2, :index])
                line.set_3d_properties(posPlot[2, :index])
                return line

            # Creating the Animation object
            line_ani = animation.FuncAnimation(fig=fig, func=update_lines,
                                               frames=int(max(posPlot.shape) / 10),
                                               fargs=None,
                                               interval=50, blit=False)

            plt.show()


            self.plotHandler.quaternion_w.buffer.put(packet_acquired[0])
            self.plotHandler.quaternion_x.buffer.put(packet_acquired[1])
            self.plotHandler.quaternion_y.buffer.put(packet_acquired[2])
            self.plotHandler.quaternion_z.buffer.put(packet_acquired[3])
            self.plotHandler.accelerometer_x.buffer.put(packet_acquired[4])  # +4 ... work around...
            self.plotHandler.accelerometer_y.buffer.put(packet_acquired[5])  # i only want to plot in
            self.plotHandler.accelerometer_z.buffer.put(packet_acquired[6])  # different plots
            self.plotHandler.gyroscope_x.buffer.put(packet_acquired[7])
            self.plotHandler.gyroscope_y.buffer.put(packet_acquired[8])
            self.plotHandler.gyroscope_z.buffer.put(packet_acquired[9])


def test():
    import sys
    from PyQt4 import QtGui
    app = QtGui.QApplication(sys.argv)
    form = QtGui.QMainWindow()
    form.resize(800, 600)
    central_widget = QtGui.QWidget(form)
    vertical_layout = QtGui.QVBoxLayout(central_widget)

    harry_plotter = IMU3DPlotter(parent=central_widget)#, app=app)
    harry_plotter.start()

    vertical_layout.addWidget(harry_plotter.plotHandler.plotWidget)
    form.setCentralWidget(central_widget)
    form.show()
    app.exec_()
    harry_plotter.stop()

if __name__ == '__main__':
    test()