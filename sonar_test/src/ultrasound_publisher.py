import rospy
import roshelper
import RPi.GPIO as GPIO
import time
from math import radians
from sensor_msgs.msg import Range


# set GPIO pin numbering
GPIO.setmode(GPIO.BCM)
# Todo: set these values with argument
triggerPin = 3
echoPin = 15

GPIO.setup(triggerPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)

# Init node
n = roshelper.Node("ultrasound", anonymous=False)

@n.publisher("/ultrasound", Range, queue_size=1)
def publish():
    """
    This calculates and publishes ultrasound range data from a ultrasound sensor.
    """
    GPIO.output(triggerPin, True)		            #Set trig0 pin as HIGH
    time.sleep(0.00001)			            #Delay of 10 us
    GPIO.output(triggerPin, False)		            #Set trig0 pin as LOW
    distance = Float('inf')

    while GPIO.input(echoPin)==0:                   #Check whether the ECHO is LOW
        pulse_start = time.time()                   #Saves the last known time of LOW pulse

        while GPIO.input(echoPin)==1:               #Check whether the ECHO is HIGH
            pulse_end = time.time()                 #Saves the last known time of HIGH pulse
        pulse_duration = pulse_end - pulse_start    #Get pulse duration to a variable

        distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
        distance = round(distance[i], 2)            #Round to two decimal points
    range = Range()
    range.radiation_type = Range.ULTRASOUND
    range.min_range = distance
    range.max_range = distance
    range.field_of_view = radians(15)
    return range

# Run with frequency of 10hz
@n.entry_point(frequency=10)
def run(): 
    publisher()

if __name__ == "__main__":
    n.start(spin=True)
