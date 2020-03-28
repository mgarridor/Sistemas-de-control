import serial

arduino=serial.Serial('/dev/ttyACM0',9600)
data_file = open("SECO_EXP_12V.txt","w+")
while(True):

    try:

        line = arduino.readline()
        data_file.write(str(line.decode('utf-8')))

        print(str(line.decode('utf8')))
    except UnicodeDecodeError:
        pass
