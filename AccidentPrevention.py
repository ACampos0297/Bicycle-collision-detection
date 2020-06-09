
#Libraries
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit


#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
#set GPIO Pins
GPIO_TRIGGER = 13
GPIO_ECHO = 22
GPIO_HALL = 4


GPIO_ECHOL = 18
GPIO_ECHOR = 17
GPIO_TRIGL = 24
GPIO_TRIGR = 23

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_HALL, GPIO.IN)

GPIO.setup(GPIO_ECHOL,GPIO.IN)
GPIO.setup(GPIO_ECHOR,GPIO.IN)
GPIO.setup(GPIO_TRIGR,GPIO.OUT)
GPIO.setup(GPIO_TRIGL,GPIO.OUT)
#for i in range(20):

maxVal = 1000

ultraStart = time.time()
dist = 0

lastTime = time.time()
lastDist = 0

incomingCount = 0

rad = 13*2.54
circM = rad*2*3.14/100
hallStart = time.time()
mps = 0

dist = 0
vel = 0
vel1 = 0
vel2 = 0
vel3 = 0

leftStart = time.time()
rightStart = time.time()

leftDet = False
rightDet = False

leftDist = 0
rightDist = 0

lastPitch = time.time()
pitch = 0

highL = 0
highR = 0

incoming = 0
offBalance = True

def output():
    global dist, vel, mps, pitch, leftDist, rightDist, incoming, offBalance
    print ("\n"*50)
    print ("Measured Distance = %.1f cm" % dist)
    print ("Incoming Velocity = %.1f m/s" % vel)
    print ("Bike Velocity = %.1f m/s" % mps)
    print ("Pitch = %.2f" % pitch)
    print ("Left: %.1f      Right: %.1f" % (leftDist,rightDist))
    if offBalance:
        print ("Bike is off balance")
        offBalance=False
    if incoming > 0 :
        incoming = incoming - 1
        print ("     INCOMING COLLISION DETECTED!     ")

def hall(pin):
    global mps, hallStart
    elapsed = time.time() - hallStart
    if elapsed > 0.1:
        mps = circM/elapsed
    hallStart = time.time()

def startRead(pin):
    global ultraStart
    ultraStart = time.time()


def stopRead(pin):
    global ultraStart, lastTime, lastDist, incomingCount, mps, vel, dist, incoming, vel1, vel2, vel3
    curTime = time.time()
    TimeElapsed = curTime - ultraStart
    #print ("%.3f" % TimeElapsed)
    dist = (TimeElapsed * 34300) / 2
    if(dist<500) :
        vel1 = vel2
        vel2 = vel3
        vel3 = (dist - lastDist)/(curTime-lastTime)/100
        vel = (vel1+vel2+vel3)/3
        if(vel<-0.5) :
            incomingCount= incomingCount+1
            if(incomingCount > 3):
                incoming = 3
        else:
            incomingCount = 0
        lastTime = curTime
        lastDist = dist
    ultraStart = time.time()
    time.sleep(.1)
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    time.sleep(0.00001)

def ultraInterupt(pin):
    if GPIO.input(GPIO_ECHO) == 1:
        startRead(pin)
    else:
        stopRead(pin)

def interuptLeft(pin):
    global leftStart,leftDist,rightDet,leftDet
    if GPIO.input(pin) == 1:
        leftStart = time.time()
    else:
        curTime = time.time()
        elapsed = curTime-leftStart
        dist = elapsed*34300/2
        if(dist<500):
            leftDist = dist
        leftDet = True
        if rightDet:
            calculatePitch()

def interuptRight(pin):
    global rightStart, rightDist, rightDet,leftDet
    if GPIO.input(pin) == 1:
        rightStart = time.time()
    else:
        curTime = time.time()
        elapsed = curTime-rightStart
        dist = elapsed*34300/2
        if dist<500:
            rightDist = dist
        rightDet = True
        if leftDet:
            calculatePitch()

def calculatePitch():
    global leftDist, rightDist, pitch, rightDet, leftDet, highL, highR, offBalance, lastPitch
    lastPitch = time.time()
    if(leftDist+rightDist>0):
        pitch = (leftDist-rightDist)/(leftDist+rightDist)
    if pitch > 0.3 :
        highR = 2
    if pitch < -0.3:
        highL = 2
    if highR > 0:
        if highL > 0:
            offBalance = True
    rightDet= False
    leftDet = False
    GPIO.output(GPIO_TRIGR, True)
    GPIO.output(GPIO_TRIGL, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGR, False)
    GPIO.output(GPIO_TRIGL, False)


if __name__ == '__main__':
    try:
        last_time = time.time()
        last_dist = maxVal
        
        vel_1 = 0
        vel_2 = 0
        vel_3 = 0
        GPIO.add_event_detect(GPIO_ECHO,GPIO.BOTH)
        GPIO.add_event_callback(GPIO_ECHO, ultraInterupt)
        GPIO.add_event_detect(GPIO_HALL,GPIO.FALLING)
        GPIO.add_event_callback(GPIO_HALL, hall)
        GPIO.add_event_detect(GPIO_ECHOL,GPIO.BOTH)
        GPIO.add_event_callback(GPIO_ECHOL,interuptLeft)
        GPIO.add_event_detect(GPIO_ECHOR,GPIO.BOTH)
        GPIO.add_event_callback(GPIO_ECHOR,interuptRight)
        time.sleep(1)
        # set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)
        GPIO.output(GPIO_TRIGL, True)
        GPIO.output(GPIO_TRIGR, True)
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)
        GPIO.output(GPIO_TRIGL, False)
        GPIO.output(GPIO_TRIGR, False)
        time.sleep(0.00001)
        while(True):
            output()
            if time.time()-lastPitch > 2:
                calculatePitch()
            if time.time()-lastTime > 2:
                GPIO.output(GPIO_TRIGGER, True)
                time.sleep(0.00001)
                GPIO.output(GPIO_TRIGGER, False)
            highL = highL - 1
            
            highR = highR - 1
            time.sleep(0.5)
        GPIO.cleanup()
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
