#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  3 08:50:48 2018

@author: catarina
"""
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
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import pandas as pd
# ------------------------------------------------------------------------------
#plt.style.use('classic')
plt.style.use('seaborn-whitegrid')
AvailableStyles = plt.style.available
# -------------------------------------------------------------------------
# Select dataset (comment in/out)

Fs = 100
filePath = 'coleta3_CalInertialAndMag.csv'
startTime = 7
calibStartTime = 11.7
calibEndTime = 13
stopTime = 55
mag_enabled = False
# -------------------------------------------------------------------------
# Import data

samplePeriod = 1.0/Fs

dataset = pd.read_csv(filePath)
time = dataset.iloc[:,0].values * samplePeriod
gyr = dataset.iloc[:,1:4].values
acc = dataset.iloc[:,4:7].values
mag = dataset.iloc[:,7:10].values

# -------------------------------------------------------------------------
# Manually frame data
indexSel1 = time > startTime
indexSel2 = time < stopTime
indexSel = indexSel1 * indexSel2

time = time[indexSel]
gyr = gyr[indexSel,:]
acc = acc[indexSel,:]
mag = mag[indexSel,:]

indexCalibStart = time > calibStartTime
indexCalibEnd = time < calibEndTime
indexCalib = indexCalibStart * indexCalibEnd

# -------------------------------------------------------------------------
# Plot data raw sensor data
plt.figure(figsize=(20,10))
plt.suptitle('Sensor Data', fontsize=14)
ax1 = plt.subplot(2+mag_enabled,1,1)
plt.grid(True)
plt.plot(time, gyr[:,0], 'r')
plt.plot(time, gyr[:,1], 'g')
plt.plot(time, gyr[:,2], 'b')
plt.title('Gyroscope')
plt.ylabel('Angular velocity (º/s)')
plt.legend(labels=['X', 'Y', 'Z'])


plt.subplot(2+mag_enabled,1,2,sharex=ax1)
plt.grid(True)
plt.plot(time, acc[:,0], 'r')
plt.plot(time, acc[:,1], 'g')
plt.plot(time, acc[:,2], 'b')
plt.title('Accelerometer')
plt.ylabel('Acceleration (g)')
plt.legend(['X', 'Y', 'Z', 'Filtered', 'Stationary'])

if mag_enabled:
    plt.subplot(3,1,3,sharex=ax1)
    plt.grid(True)
    plt.plot(time, mag[:,0], 'r')
    plt.plot(time, mag[:,1], 'g')
    plt.plot(time, mag[:,2], 'b')
    plt.title('Magnetrometer')
    plt.ylabel('Magnetic Flux Density  (G)')
    plt.legend(['X', 'Y', 'Z'])

plt.xlabel('Time (s)')
plt.xlim(10,13)
plt.ylim(0,0.4)

# -------------------------------------------------------------------------
# Detect stationary periods

# Compute accelerometer magnitude
acc_mag = np.sqrt(acc[:,0]**2 + acc[:,1]**2 + acc[:,2]**2)

# HP filter accelerometer data
filtCutOff = 0.001
[b, a] = signal.butter(1, (2*filtCutOff)/(1/samplePeriod), 'high')
acc_mag = signal.filtfilt(b, a, acc_mag)

# Compute absolute value
acc_mag = abs(acc_mag)

# LP filter accelerometer data
filtCutOff = 5
[b, a] = signal.butter(1, (2*filtCutOff)/(1/samplePeriod), 'low')
acc_magFilt = signal.filtfilt(b, a, acc_mag)

# Threshold detection
calib_data = acc_magFilt[indexCalib]
statistical_stationary_threshold = np.mean(calib_data) + 2*np.std(calib_data)

print('Limiar Calculado = %.4f + 2 * %.4f = %.4f' % (np.mean(calib_data),
                                                     np.std(calib_data),
                                                     statistical_stationary_threshold))

stationary = acc_magFilt < statistical_stationary_threshold

#%% -------------------------------------------------------------------------
# Plot desired outcome data
plt.figure(figsize=(20,10))
plt.grid(True)
plt.plot(time, acc_mag, ':b', label='Acceleration')
plt.plot(time, acc_magFilt, 'g', label='Filtered')
#plt.plot(time, stationary.astype(np.uint8), 'k', label='Stationary')
plt.plot(time, statistical_stationary_threshold*np.ones(time.shape), ':k', label='Threshold')
plt.title('Moviment Detection')
plt.ylabel('Acceleration (g)')
plt.legend()
plt.xlabel('Time (s)')
plt.xlim(10,13)
plt.ylim(0,0.4)


#%% -------------------------------------------------------------------------
# LP filter accelerometer data using MVA
filtCutOff = 5
filtCutOff_nyq_norm = filtCutOff/(Fs/2.0)
N = np.sqrt(0.196202+filtCutOff_nyq_norm**2)/filtCutOff_nyq_norm
N = np.round(N).astype(int)
N = 8
mva_window = np.ones((N,1))
mva_window = mva_window/np.sum(mva_window)

signal_window = np.zeros((N,1))
acc_magFilt_mva = np.zeros(acc_mag.shape)
for n in  range(len(acc_mag)):
    signal_window[0:-1] = signal_window[1:]
    signal_window[-1] = acc_mag[n]
    acc_magFilt_mva[n] = np.sum(signal_window * mva_window)

# Threshold detection
calib_data = acc_magFilt_mva[indexCalib]
statistical_stationary_threshold = np.mean(calib_data) + 2*np.std(calib_data)

print('Limiar Calculado = %.4f + 2 * %.4f = %.4f' % (np.mean(calib_data),
                                                     np.std(calib_data),
                                                     statistical_stationary_threshold))

stationary = acc_magFilt_mva < statistical_stationary_threshold
#%% -------------------------------------------------------------------------
# Corrigindo o atraso, mas isso faz com que eu so tenha certeza de algo
# que ocorreu a N/2 amostras atras da minha amostra atual
acc_magFilt_mva_no_delay = acc_magFilt_mva.copy()
acc_magFilt_mva_no_delay[:-int(N/2)] = acc_magFilt_mva_no_delay[int(N/2):]

#%% -------------------------------------------------------------------------
# Plot desired outcome data

plt.figure(figsize=(20,10))
plt.grid(True)
plt.plot(time, acc_mag, ':b', label='Acceleration')
plt.plot(time, acc_magFilt, 'g', label='Desired')
plt.plot(time, acc_magFilt_mva, 'r', label='MVA')
#plt.plot(time, acc_magFilt_mva_no_delay, 'purple', label='MVA deslocada')
#plt.plot(time, stationary.astype(np.uint8), 'k', label='Stationary')
plt.plot(time, statistical_stationary_threshold*np.ones(time.shape), ':k',
         label='Threshold')
plt.title('Moviment Detection')
plt.ylabel('Acceleration (g)')
plt.legend()
plt.xlabel('Time (s)')
plt.xlim(10,13)
plt.ylim(0,0.4)



#%% -------------------------------------------------------------------------
## Efeito do tamanho da janela da media movel
#acc_magFilt_N = {}
#for N in range(4,7):
#    mva_window = np.zeros((N,1))
#
#    acc_magFilt_mva = np.zeros(acc_mag.shape)
#    for n in  range(len(acc_mag)):
#        mva_window[0:-1] = mva_window[1:]
#        mva_window[-1] = acc_mag[n]
#        acc_magFilt_mva[n] = np.mean(mva_window)
#    
#    acc_magFilt_N[N] = acc_magFilt_mva
#
## -------------------------------------------------------------------------
## Plot desired outcome data
#plt.figure(figsize=(20,10))
#plt.grid(True)
#plt.plot(time, acc_mag, 'b', label='Acceleration')
#plt.plot(time, acc_magFilt, 'g', label='Desired')
#for key, item in acc_magFilt_N.items():
#    plt.plot(time, item, label=str(key))
#plt.title('Moviment Detection')
#plt.ylabel('Acceleration (g)')
#plt.legend()
#plt.xlabel('Time (s)')
#plt.xlim(10,13)
#plt.ylim(0,0.4)
#

#%% -------------------------------------------------------------------------
# LP filter accelerometer data using Hamming Window
N = 8
ham_window = signal.windows.hamming(N)
ham_window = ham_window/np.sum(ham_window)

signal_window = np.zeros((N,1))
a = [1, 2, 3, 4]

acc_magFilt_ham = np.zeros(acc_mag.shape)
for n in  range(len(acc_mag)):
    signal_window[0:-1] = signal_window[1:]
    signal_window[-1] = acc_mag[n]
    acc_magFilt_ham[n] = np.sum(signal_window * ham_window)/N
    
# Threshold detection
calib_data = acc_magFilt_ham[indexCalib]
statistical_stationary_threshold = np.mean(calib_data) + 2*np.std(calib_data)

print('Limiar Calculado = %.4f + 2 * %.4f = %.4f' % (np.mean(calib_data),
                                                     np.std(calib_data),
                                                     statistical_stationary_threshold))

stationary = acc_magFilt_ham < statistical_stationary_threshold

#%% -------------------------------------------------------------------------
# Plot desired outcome data
plt.figure(figsize=(20,10))
plt.grid(True)
plt.plot(time, acc_mag, ':b', label='Acceleration')
plt.plot(time, acc_magFilt, 'g', label='Desired')
plt.plot(time, acc_magFilt_ham, 'r', label='HAM')
#plt.plot(time, stationary.astype(np.uint8), 'k', label='Stationary')
plt.plot(time, statistical_stationary_threshold*np.ones(time.shape), ':k',
         label='Threshold')
plt.title('Moviment Detection')
plt.ylabel('Acceleration (g)')
plt.legend()
plt.xlabel('Time (s)')
plt.xlim(10,13)
plt.ylim(0,0.4)

#%% -------------------------------------------------------------------------
# LP filter accelerometer data using blackman Window
N = 8
bm_window = signal.windows.blackman(N)
bm_window = bm_window/np.sum(bm_window)

signal_window = np.zeros((N,1))
a = [1, 2, 3, 4]

acc_magFilt_bm = np.zeros(acc_mag.shape)
for n in  range(len(acc_mag)):
    signal_window[0:-1] = signal_window[1:]
    signal_window[-1] = acc_mag[n]
    acc_magFilt_bm[n] = np.sum(signal_window * bm_window)/N
    
# Threshold detection
calib_data = acc_magFilt_bm[indexCalib]
statistical_stationary_threshold = np.mean(calib_data) + 2*np.std(calib_data)

print('Limiar Calculado = %.4f + 2 * %.4f = %.4f' % (np.mean(calib_data),
                                                     np.std(calib_data),
                                                     statistical_stationary_threshold))

stationary = acc_magFilt_bm < statistical_stationary_threshold

#%% -------------------------------------------------------------------------
# Plot desired outcome data
plt.figure(figsize=(20,10))
plt.grid(True)
plt.plot(time, acc_mag, ':b', label='Acceleration')
plt.plot(time, acc_magFilt, 'g', label='Desired')
plt.plot(time, acc_magFilt_bm, 'r', label='Blackman')
#plt.plot(time, stationary.astype(np.uint8), 'k', label='Stationary')
plt.plot(time, statistical_stationary_threshold*np.ones(time.shape), ':k',
         label='Threshold')
plt.title('Moviment Detection')
plt.ylabel('Acceleration (g)')
plt.legend()
plt.xlabel('Time (s)')
plt.xlim(10,13)
plt.ylim(0,0.4)

#%% window tested

plt.figure(figsize=(20,10))
plt.grid(True)
plt.plot(mva_window,'-or', label='MVA')
plt.plot(ham_window, '-og', label='HAM')
plt.plot(bm_window, '-ob', label='Blackman')
plt.title('Filtering Windows')
plt.ylabel('Value')
plt.legend()
plt.xlabel('N')