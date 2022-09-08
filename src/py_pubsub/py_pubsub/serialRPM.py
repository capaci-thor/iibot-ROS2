import serial, time
puertoSerial = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)   #Espera 2 segundos para conectarse al puerto serial

while 1:
    try:
        datos = puertoSerial.readline()
        print(datos)
    
    except KeyboardInterrupt:
        break

puertoSerial.close()