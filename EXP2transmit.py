# !/usr/bin/python

import Adafruit_BBIO.GPIO as GPIO
import time
import datetime
 
GPIO.setup("P8_8", GPIO.OUT)
GPIO.setup("P8_10",GPIO.IN)
LED0 = "USR0"
LED1 = "USR1"
LED2 = "USR2"
LED3 = "USR3"
GPIO.setup(LED0, GPIO.OUT)
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)
GPIO.setup(LED3, GPIO.OUT)
GPIO.output(LED0, GPIO.LOW)
GPIO.output(LED1, GPIO.LOW)
GPIO.output(LED2, GPIO.LOW)
GPIO.output(LED3, GPIO.LOW)

import serial
# from serial import serial

# initialization and open the port
# possible timeout values:
# 1. None: wait forever, block call
# 2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

ser = serial.Serial()

# ser.port = "/dev/ttyUSB0"
ser.port = "/dev/ttyO4"
# ser.port = "/dev/ttyS2"

ser.baudrate = 9600
ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser.parity = serial.PARITY_NONE  # set parity check: no parity
ser.stopbits = serial.STOPBITS_ONE  # number of stop bits

# ser.timeout = None          #block read
ser.timeout = 1  # non-block read
# ser.timeout = 2              #timeout block read

ser.xonxoff = False  # disable software flow control
ser.rtscts = False  # disable hardware (RTS/CTS) flow control
ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control

ser.writeTimeout = 2  # timeout for write

try:
    ser.open()
except Exception as e:
    print("error open serial port: " + str(e))
    exit()

if ser.isOpen():
    try:
        ser.flushInput()  # flush input  buffer
        ser.flushOutput()  # flush output buffer
    except Exception.e1:
        print("error to open serial port")

time.sleep(0.2)  # give the serial port sometime to receive the data
ATcommand="AT+ADDRESS=1\r\n" #set address
ser.write(ATcommand)
time.sleep(0.2)
ATcommand="AT+CRFOP=5\r\n" #set power
ser.write(ATcommand)
time.sleep(0.2)
ATcommand="AT+PARAMETER=12,7,1,4\r\n"
ser.write(ATcommand)
time.sleep(0.2)
ser.flushInput()

def newATcommand(SpreadFactor, Bandwidth, CRC, Power,syncTime):
    ts=time.time()
    while ts<syncTime:
        ts=time.time()
        time.sleep(0.1)
    ATcommand="AT+PARAMETER=" + str(SpreadFactor) + "," + str(Bandwidth) + "," + str(CRC) + ",4\r\n"
    ser.write(ATcommand)
    #print(ATcommand)
    time.sleep(0.5)
    ATcommand="AT+CRFOP=" + str(Power) + "\r\n"
    ser.write(ATcommand)
    #print(ATcommand)
    #file = open("EXP2transmitdata.txt","a")
    #ile.write("NEW PARAMETERS,Spread Factor: " + str(SpreadFactor)+ ", BANDWIDTH: " + str(Bandwidth) + ", CODING RATE: " + str(CRC) + ", POWER: " + str(Power) +"\r\n")
    #file.close()
    time.sleep(0.5)
    ser.flushInput()
def runTransmission(transmitTime,timetoTransmit):
    timer1=time.time()+60*timetoTransmit
    n=0
    while True:
        if time.time() > timer1:
            break
        message1="AT+SEND=2,62,Hello my name is Andrew. This message is being sent on repeat.\r\n"
        ser.write(message1)
        n+=1
        ts=time.time()
        st=datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        file = open("EXP1Transmitdata.txt","a")
        file.write(st+ ","+str(n))
        file.write("," + message1)
        file.close()
        timer2=time.time()+transmitTime
        while time.time()<timer2 and time.time() < timer1:
            time.sleep(0.01)
        #time.sleep(transmitTime)
            
def transmit(transmitTime,timetoTransmit,syncTime):
    GPIO.output(LED0, GPIO.HIGH)
    parameterInterval=620
    newATcommand(8,3,1,15,syncTime) #spread factor, bandwidth, CRC, Power
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,3,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,3,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,5,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,5,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,5,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,7,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,7,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,7,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,9,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,9,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,9,1,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
        
    GPIO.output(LED1, GPIO.HIGH)
    newATcommand(8,3,4,15,syncTime) #spread factor, bandwidth, CRC, Power
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,3,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,3,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,5,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,5,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,5,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,7,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,7,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,7,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,9,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,9,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,9,4,15,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    GPIO.output(LED2, GPIO.HIGH)
    
    newATcommand(8,3,1,0,syncTime) #spread factor, bandwidth, CRC, Power
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,3,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,3,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,5,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,5,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,5,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,7,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,7,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,7,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,9,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,9,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,9,1,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    GPIO.output(LED3, GPIO.HIGH)
    
    newATcommand(8,3,4,0,syncTime) #spread factor, bandwidth, CRC, Power
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,3,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,3,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,5,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,5,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,5,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,7,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,7,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,7,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    newATcommand(8,9,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(10,9,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    newATcommand(12,9,4,0,syncTime)
    runTransmission(transmitTime,timetoTransmit)
    syncTime+=parameterInterval
    
    GPIO.output(LED0, GPIO.LOW)
    GPIO.output(LED1, GPIO.LOW)
    GPIO.output(LED2, GPIO.LOW)
    GPIO.output(LED3, GPIO.LOW)

syncTime=1584895000
ts=time.time()
while ts < syncTime:
    ts=time.time()
    print(ts)
    time.sleep(1)


#print("Start command sent.")
while True:
    GPIO.output(LED0, GPIO.LOW)
    GPIO.output(LED1, GPIO.LOW)
    GPIO.output(LED2, GPIO.LOW)
    GPIO.output(LED3, GPIO.LOW)
    transmitTime=30 #transmit every 30 seconds
    timetoTransmit=10 #transmit at each level for 10 minutes
    transmit(transmitTime,timetoTransmit,syncTime)
    break

else:
    print("can't open serial port")