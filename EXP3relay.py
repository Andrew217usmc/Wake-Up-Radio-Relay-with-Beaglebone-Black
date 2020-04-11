import Adafruit_BBIO.GPIO as GPIO
import time
import serial
import datetime
import statistics

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
        
    n=0
    time.sleep(0.5)  # give the serial port sometime to receive the data
    
    ATcommand="AT+ADDRESS=2\r\n" #set transceiver to address 2
    ser4.write(ATcommand)
    time.sleep(0.4)
    ATcommand="AT+CRFOP=1\r\n" #set power of transceiver to 1 dBm
    ser4.write(ATcommand)
    time.sleep(0.4)
    ATcommand="AT+PARAMETER=12,7,1,4\r\n"
    ser4.write(ATcommand)
    time.sleep(0.4)
    ser4.write("AT+MODE=0\r\n")
    time.sleep(0.4)
    ser4.flushInput()
    
    def newATcommand(SpreadFactor, Bandwidth, CRC, Power, syncTime):
        ts=time.time()
        while ts<syncTime:
            ts=time.time()
            time.sleep(0.1)
        file = open("RELAYdata.txt","a")
        ATcommand="AT+PARAMETER=" + str(SpreadFactor) + "," + str(Bandwidth) + "," + str(CRC) + ",4\r\n"
        print(ATcommand)
        file.write("NEW PARAMETERS,Spread Factor: " + str(SpreadFactor)+ ", BANDWIDTH: " + str(Bandwidth) + ", CODING RATE: " + str(CRC) + ", POWER: " + str(Power) +". This power maintained at 15 dBm.\r\n")
        ser4.write(ATcommand)
        time.sleep(0.5)

        ser4.flushInput()
        ATcommand="AT+CRFOP=15\r\n"
        print(ATcommand)
        ser4.write(ATcommand)
        time.sleep(0.5)
        ser4.flushInput()
        file.close()

    def relayingTransmission(timetoREC,averageTransmitTime):
        establishedTimeInterval=False
        n=0
        timer1=time.time()+60*timetoREC
        while True:
            if establishedTimeInterval==False:
                averageTransmitTime = findAverageTime(timer1)
                establishedTimeInterval=True
            if time.time() > timer1:
                break
            file = open("RELAYdata.txt","a")
            receivedMessage = ser4.readline();
            while receivedMessage == "":
                receivedMessage = ser4.readline();
                if time.time() > timer1:
                    ser4.flushInput()
                    break
            print(receivedMessage)
            n+=1
            y=0
            firstComma=receivedMessage.find(",")
            secondComma=receivedMessage.find(",",firstComma+1)
            thirdComma=receivedMessage.find(",",secondComma+1)
            fourthComma=receivedMessage.find(",",thirdComma+1)
            actualMessage=receivedMessage[secondComma+1:thirdComma]
            rssi=receivedMessage[thirdComma+2:fourthComma]
            lengthofMSG = len(actualMessage)
            ts=time.time()
            st=datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
            file.write(st+ ",")
            file.write(str(n))
            file.write("," + receivedMessage)
            rssi2=0
            try:
                rssi2=int(rssi)
            except ValueError:
                y=1
            if rssi2 > 70 and y==0:
                ATcommand="AT+ADDRESS=3\r\n" #set transceiver to address 3
                ser4.write(ATcommand)
                time.sleep(2)
                ser4.flushInput()
                file.write("Message retransmitted at 15 dBm. RSSI below -70 dBm.\r\n")
                ser4.write("AT+SEND=2," + str(lengthofMSG) +"," + actualMessage + "\r\n")
                time.sleep(3)
                ser4.flushInput()
                ATcommand="AT+ADDRESS=2\r\n" #set transceiver to address 2
                ser4.write(ATcommand)
                time.sleep(0.5)
                ser4.flushInput()
            file.close()
            ser4.write("AT+PARAMETER?\r\n")
            rememberParameters = ser4.readline();
            while receivedMessage == "":
                rememberParameters = ser4.readline();
                if time.time() > timer1:
                    ser4.flushInput()
                    break
            ser4.write("AT+MODE=1\r\n")
            print("Transceiver asleep.")
            time.sleep(averageTransmitTime-10)# sleep for 10 seconds less than calculated average time interval
            ser4.write("AT+MODE=0\r\n")
            print("Transceiver awake.")
            time.sleep(0.5)
            ser4.write("AT" + rememberParameters + "\r\n")
            time.sleep(0.5)
            ser4.flushInput()
    
    def findAverageTime(timer1):
        goodstdDev=False
        while goodstdDev==False:
            receivedTimes = [] #create blank list to hold time between messages received
            gotfirstMSG=False
            messageCount=0
            while messageCount < 4:
                while gotfirstMSG==False: #This loop runs first and runs once to get the first message
                    print("Ready for first message.")
                    receivedMessage = ser4.readline();
                    while receivedMessage == "":
                        receivedMessage = ser4.readline();
                        if time.time() > timer1: #if received timer is exceeded, break out of loop
                            ser4.flushInput()
                            break
                    recMSGtime=time.time()
                    print("Got first message")
                    gotfirstMSG=True
                receivedMessage = ser4.readline();
                while receivedMessage == "": #wait for message to come in
                    receivedMessage = ser4.readline();
                    if time.time() > timer1:
                        ser4.flushInput()
                        break
                timeNow=time.time()
                x=timeNow-recMSGtime
                recMSGtime=timeNow
                receivedTimes.append(int(x))
                messageCount+=1
                print(x)
            averageTime = (sum(receivedTimes)/len(receivedTimes))
            standardDev=statistics.stdev(receivedTimes)
            if standardDev < 1: #this checks if you have outliers in your data set. Increase to allow for more variation. 
                goodstdDev=True
            else: #if standard deviation isn't good, sorts the list and removes the highest number. Calculates average time and standard deviation again. This allows for one outlier in the data set.
                receivedTimes.sort(reverse=True)
                del receivedTimes[0]
                averageTime = (sum(receivedTimes)/len(receivedTimes))
                standardDev=statistics.stdev(receivedTimes)
                if standardDev < 1: #checks standard deviation again.
                    goodstdDev=True
            print(averageTime)
            print(standardDev)
        file = open("RELAYdata.txt","a")
        file.write("Established transmission interval: " + str(averageTime) + "seconds\r\n")
        file.close()
        return averageTime
    
    def relay(timetoREC,syncTime,averageTransmitTime):
        GPIO.output(LED0, GPIO.HIGH)
        parameterInterval=620
        newATcommand(8,3,1,15,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,3,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,3,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,5,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,5,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,5,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,7,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,7,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,7,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,9,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,9,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,9,1,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        GPIO.output(LED1, GPIO.HIGH)
        newATcommand(8,3,4,15,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,3,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,3,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,5,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,5,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,5,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,7,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,7,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,7,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,9,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,9,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,9,4,15,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        GPIO.output(LED2, GPIO.HIGH)
        
        newATcommand(8,3,1,0,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,3,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,3,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,5,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,5,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,5,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,7,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,7,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,7,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,9,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,9,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,9,1,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        GPIO.output(LED3, GPIO.HIGH)
        
        newATcommand(8,3,4,0,syncTime) #spread factor, bandwidth, CRC, Power
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,3,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,3,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,5,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,5,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,5,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,7,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,7,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,7,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        newATcommand(8,9,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(10,9,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        newATcommand(12,9,4,0,syncTime)
        relayingTransmission(timetoREC,averageTransmitTime)
        syncTime+=parameterInterval
        
        GPIO.output(LED0, GPIO.LOW)
        GPIO.output(LED1, GPIO.LOW)
        GPIO.output(LED2, GPIO.LOW)
        GPIO.output(LED3, GPIO.LOW)
    
    syncTime=1585520000
    ts=time.time()
    while ts < syncTime: #waiting for start time/date
        ts=time.time()
        print(ts)
        time.sleep(1)
        
    averageTransmitTime=0
    while True:
        timetoREC=10
        relay(timetoREC,syncTime,averageTransmitTime)

        break

else:
    print("can't open serial port")