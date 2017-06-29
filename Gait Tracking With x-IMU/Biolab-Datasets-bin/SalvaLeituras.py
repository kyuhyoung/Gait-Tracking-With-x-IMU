import serial
from datetime import datetime

arduinoData = serial.Serial()
arduinoData.port = '/dev/rfcomm0'
arduinoData.baudrate = 115200
arduinoData.timeout = 1
arduinoData.open()

strikes = 0;

horario = datetime.now()
horario_anterior = horario
arq = open( "leituras_" + str(horario).split()[0] + '.binlog', 'w')

arduinoData.flushInput()
while True:
    arduinoString = arduinoData.readline()
    horario = datetime.now()
    arq.write(arduinoString)
    print(arduinoString)
    if horario.second != horario_anterior.second:
    	print(arduinoString)
    	if len(arduinoString) == 0:
    		strikes = strikes + 1
    		if strikes > 3:
    			break
    		horario_anterior = horario
    	
arduinoData.close()
arq.close()
