import zenoh, time
from cognifly import Cognifly

def identify_listener(sample):
    cf = Cognifly(drone_hostname="my_drone_name.local")
    cf.arm()
    time.sleep(2)
    cf.disarm()

if __name__ == "__main__":
    session = zenoh.open()
    sub = session.declare_subscriber('identify', identify_listener)
    while True:
        time.sleep(1)
