import serial
import time
from datetime import datetime


SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD_RATE = 115200

save_dT = 60*10 #save data every 10 minutes




#my_file = open("Log.txt","w+")






serialPort = serial.Serial(SERIAL_PORT)
serialPort.baudrate = SERIAL_BAUD_RATE








# ser = serial.Serial('dev/ttyUSB0',
#                     baudrate=9600,
# 					parity=serial.PARITY_NONE,
# 					stopbits=serial.STOPBITS_ONE)




# while(1):
#     data = ser.readline()
#     try:
#         #print(data.decode('Ascii'))
#         print(data)
#     except UnicodeDecodeError:
#         print("decode error")





tmpA = []


save_timeStamp = time.time()





def getTemp1(line):

    words = line.split(" ")
   
    return float(words[1])

def getTemp2(line):

    words = line.split(" ")
   
    return float(words[3])


def getPress(line):

    words = line.split(" ")
    return float(words[5])


def getHumidity(line):
    words = line.split(" ")
    return float(words[7])




def append_to_file():

    now = datetime.now()
    
    name = str(now.day) + "_" + str(now.month) + "_" + str(now.year) + ".txt"

    print(name)


    my_file = open("Logs/" + name ,"a+")

    for elem in tmpA:
        my_file.write(elem)


    tmpA.clear()


while(1):
    # Wait until there is data waiting in the serial buffer


    time.sleep(0.1)

    if(serialPort.in_waiting > 0):
        # Read data out of the buffer until a carraige return / new line is found
        serialString = serialPort.readline()

        try:
            now = datetime.now()

            S = serialString.decode('Ascii')
            temperature1 = getTemp1(S)
            presure = getPress(S)
            temperature2 = getTemp2(S)
            humidity = getHumidity(S)

            timestamp = now.strftime("%H:%M:%S")


            toSave = timestamp + " " + str(temperature1) + " " + str(temperature2) + " " + str(presure) + " " + str(humidity) + " \n"

            tmpA.append(toSave)
            print(toSave)




        except BaseException as e:
            print(e)
        # Print the contents of the serial data
       



    if(len(tmpA) > 1):
        append_to_file()


        # Tell the device connected over the serial port that we recevied the data!
        # The b at the beginning is used to indicate bytes!