import zenoh, time
import RPi.GPIO as GPIO
from cognifly import Cognifly

cf = Cognifly(drone_hostname="Cognifly1.lan", gui=False)


def identify_listener(sample):
    global cf
    global position
    position = cf.get_position(sample)

import time, zenoh
import RPi.GPIO as GPIO
session = zenoh.open()
session.declare_subscriber ('cognifly_id', led_light)
is_called = False
def led_light():
    global is_called
    if is_called: return
    is_called = True

    GPIO.output(26,GPIO.HIGH)
    print ("LED is ON")
    time.sleep(1)
    GPIO.output(26,GPIO.LOW)
    print ("LED is OFF")
    is_called = False

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
