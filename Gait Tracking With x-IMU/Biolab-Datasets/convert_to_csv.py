#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys 

print("*"*80)

file_name = str(sys.argv[1])
print("Convertendo arquivo: " + str(file_name))

file_dataout = open(file_name.replace('.log','_CalInertialAndMag.csv') ,'w')
file_datain = open(file_name, 'r')

file_dataout.write('Packet number,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g),Magnetometer X (G),Magnetometer Y (G),Magnetometer Z (G)\n')

Magnect_fake_data = '0.00,0.00,0.00'
packet_num = 0
for line in file_datain:
	ax = line.split()[0]
	ay = line.split()[1]
	az = line.split()[2]
	gx = line.split()[3]
	gy = line.split()[4]
	gz = line.split()[5]
	packet_num = packet_num + 1
	file_dataout.write("%d,%s,%s,%s,%s,%s,%s,%s\n" %(packet_num, gx,gy,gz,ax,ay,az,Magnect_fake_data))
	
file_datain.close()
file_dataout.close()

print("%d escritas em: %s" % (packet_num, file_name.replace('.log','.csv')))
print("%.2f segundos (%.2f min) de coleta" % (packet_num/100.,packet_num/6000.))
print("Convertido com sucesso!")
print("*"*80)
