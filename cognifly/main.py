import zenoh, time
from cognifly import Cognifly

cf = Cognifly(drone_hostname="Cognifly1.lan", gui=False)

def identify_listener(sample):
    global cf
    global position
    position = cf.get_position(sample)

if __name__ == "__main__":
    global position
    session = zenoh.open()
    sub = session.declare_subscriber('identify', identify_listener)
    while True:
        time.sleep(1)
        print("position is: ", position )
