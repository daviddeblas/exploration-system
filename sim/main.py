import zenoh, time
import rospy
from geometry_msgs.msg import Twist
MOVE_TIME_SEC = 0.01

move = Twist()

def listener(sample):
    print("Listen")
    message = sample.payload.decode('utf-8')
    print(message)
    move.linear.x = 10.0
    time.sleep(MOVE_TIME_SEC)
    move.linear.x = 0.0

if __name__ == "__main__":
    session = zenoh.open()
    sub = session.declare_subscriber('identify', listener)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('movement')

    print("Started listening")
    while True:
        time.sleep(1)
