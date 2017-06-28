#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys 

print("*"*80)

file_name = str(sys.argv[1])
print("Convertendo arquivo: " + str(file_name))

file_out = open(file_name.replace('.log','.csv') ,'w')
file_in = open(file_name, 'r')

file_out.write('Packet number,Gyroscope X (deg/s),Gyroscope Y (deg/s),Gyroscope Z (deg/s),Accelerometer X (g),Accelerometer Y (g),Accelerometer Z (g)\n')

packet_num = 0
for line in file_in:
	ax = line.split()[0]
	ay = line.split()[1]
	az = line.split()[2]
	gx = line.split()[3]
	gy = line.split()[4]
	gz = line.split()[5]
	packet_num = packet_num + 1
	file_out.write("%d,%s,%s,%s,%s,%s,%s\n" %(packet_num, gx,gy,gz,ax,ay,az)
	
file_in.close()
file_out.close()

print("%d escritas em: %s" % (packet_num, file_name.replace('.log','.csv'))
print("Convertido com sucesso!")
print("*"*80)
