import Adafruit_BBIO.GPIO as GPIO
import time
import serial
import datetime

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

ser4 = serial.Serial()

ser4.port = "/dev/ttyO4"

ser4.baudrate = 9600
ser4.bytesize = serial.EIGHTBITS  # number of bits per bytes
ser4.parity = serial.PARITY_NONE  # set parity check: no parity
ser4.stopbits = serial.STOPBITS_ONE  # number of stop bits

ser4.timeout = 1  # non-block read

ser4.xonxoff = False  # disable software flow control
ser4.rtscts = False  # disable hardware (RTS/CTS) flow control
ser4.dsrdtr = False  # disable hardware (DSR/DTR) flow control

ser4.writeTimeout = 2  # timeout for write

try:
    ser4.open()
except Exception as e:
    print("error open serial port: " + str(e))
    exit()

if ser4.isOpen():
    try:
        ser4.flushInput()  # flush input  buffer
        ser4.flushOutput()  # flush output buffer
    except Exception.e1:
        print("error to open serial port")

    time.sleep(0.5)  # give the serial port sometime to receive the data
    
    ATcommand="AT+ADDRESS=4\r\n" #set receiver to address 4
    ser4.write(ATcommand)
    time.sleep(0.2)
    ATcommand="AT+CRFOP=1\r\n" #set power of receiver
    ser4.write(ATcommand)
    time.sleep(0.2)
    ATcommand="AT+PARAMETER=12,7,1,4\r\n"
    ser4.write(ATcommand)
    time.sleep(0.2)
    ser4.flushInput()
    
    def newATcommand(SpreadFactor, Bandwidth, CRC, Power,syncTime):
        ts=time.time()
        while ts<syncTime:
            ts=time.time()
            time.sleep(0.1)
        file = open("RECEIVERdata.txt","a")
        ATcommand="AT+PARAMETER=" + str(SpreadFactor) + "," + str(Bandwidth) + "," + str(CRC) + ",4\r\n"
        print(ATcommand)
        file.write("NEW PARAMETERS,Spread Factor: " + str(SpreadFactor)+ ", BANDWIDTH: " + str(Bandwidth) + ", CODING RATE: " + str(CRC) + ", POWER: " + str(Power) +"\r\n")
        ser4.write(ATcommand)
        time.sleep(0.5)
        ser4.flushInput()
        ATcommand="AT+CRFOP=" + str(Power) + "\r\n"
        print(ATcommand)
        ser4.write(ATcommand)
        time.sleep(0.5)
        ser4.flushInput()
        file.close()

    def relayingTransmission(timetoREC):
        n=0
        timer1=time.time()+60*timetoREC 
        while True:
            if time.time() > timer1:
                break
            file = open("RECEIVERdata.txt","a")
            receivedMessage = ser4.readline();
            while receivedMessage == "":
                receivedMessage = ser4.readline();
                if time.time() > timer1:
                    ser4.flushInput()
                    break
            print(receivedMessage)
            n+=1
            firstComma=receivedMessage.find(",")
            secondComma=receivedMessage.find(",",firstComma+1)
            thirdComma=receivedMessage.find(",",secondComma+1)
            fourthComma=receivedMessage.find(",",thirdComma+1)
            actualMessage=receivedMessage[secondComma+1:thirdComma]
            lengthofMSG = len(actualMessage)
            ts=time.time()
            st=datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            print(st)
            file.write(st+ ",")
            file.write(str(n))
            file.write("," + receivedMessage)
            file.close()
    def relay(timetoREC,syncTime):
        GPIO.output(LED0, GPIO.HIGH)
        parameterInterval=200
        newATcommand(8,3,1,15,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,3,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,3,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,5,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,5,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,5,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,7,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,7,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,7,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,9,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,9,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,9,1,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        GPIO.output(LED1, GPIO.HIGH)
        newATcommand(8,3,4,15,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,3,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,3,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,5,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,5,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,5,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,7,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,7,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,7,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,9,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,9,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,9,4,15,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        GPIO.output(LED2, GPIO.HIGH)
        
        newATcommand(8,3,1,0,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,3,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,3,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,5,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,5,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,5,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,7,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,7,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,7,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,9,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,9,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,9,1,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        GPIO.output(LED3, GPIO.HIGH)
        
        newATcommand(8,3,4,0,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,3,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,3,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,5,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,5,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,5,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,7,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,7,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,7,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        newATcommand(8,9,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(10,9,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        newATcommand(12,9,4,0,syncTime)
        relayingTransmission(timetoREC)
        syncTime+=parameterInterval
        
        GPIO.output(LED0, GPIO.LOW)
        GPIO.output(LED1, GPIO.LOW)
        GPIO.output(LED2, GPIO.LOW)
        GPIO.output(LED3, GPIO.LOW)
    
    syncTime=1584815250
    ts=time.time()
    while ts < syncTime:
        ts=time.time()
        print(ts)
        time.sleep(1)
            
    while True:
        timetoREC=2.7
        relay(timetoREC,syncTime)
        timetoREC=2.8
        relay(timetoREC,syncTime)
        timetoREC=2.9
        relay(timetoREC,syncTime)
        timetoREC=3
        relay(timetoREC,syncTime)
        break

else:
    print("can't open serial port")