import RPi.GPIO as GPIO # import GPIO library
import time                                     # import time library
import rospy
import roshelper
from sensor_msgs.msg import Range

# set GPIO pin numbering
GPIO.setmode(GPIO.BCM)

trig[0] = 3
trig[1] = 5
trig[2] = 7
trig[3] = 8
trig[4] = 10
trig[5] = 11
trig[6] = 12
trig[7] = 13
echo[0] = 15
echo[1] = 16
echo[2] = 18
echo[3] = 19
echo[4] = 21
echo[5] = 22
echo[6] = 23
echo[7] = 24

for i in range(0, 7):
    GPIO.setup(trig[i],GPIO.OUT)                    #Set pin as GPIO out
    GPIO.setup(echo[i],GPIO.IN)                     #Set pin as GPIO in
    GPIO.output(trig[i], False)                     #Set trig0 pin as LOW

    GPIO.output(trig[i], True)		            #Set trig0 pin as HIGH
    time.sleep(0.00001)			            #Delay of 10 us
    GPIO.output(trig[i], False)		            #Set trig0 pin as LOW

    while GPIO.input(echo[i])==0:                   #Check whether the ECHO is LOW
        pulse_start = time.time()                   #Saves the last known time of LOW pulse

        while GPIO.input(echo[i])==1:               #Check whether the ECHO is HIGH
            pulse_end = time.time()                 #Saves the last known time of HIGH pulse
        pulse_duration = pulse_end - pulse_start    #Get pulse duration to a variable

        distance[i] = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
        distance[i] = round(distance[i], 2)            #Round to two decimal points
