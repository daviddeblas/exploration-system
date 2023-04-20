import time, zenoh
import RPi.GPIO as GPIO

session = zenoh.open()
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

light = session.declare_subscriber ('cognifly_id', led_light)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(26,GPIO.OUT)
    while True:
        time.sleep(1)
