import zenoh
import time
import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_limo1', anonymous=False)
rate = rospy.Rate(10)
move = Twist()
in_mission = False


def start_listener(sample):
    global in_mission
    message = sample.payload.decode('utf-8')
    print(message)
    move.linear.x = -0.5
    move.angular.z = -0.5
    pub.publish(move)
    in_mission = True


def identify_listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)


def finish_listener(sample):
    global in_mission
    message = sample.payload.decode('utf-8')
    print(message)
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    in_mission = False


if __name__ == "__main__":
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    session = zenoh.open()
    sub1 = session.declare_subscriber('start', start_listener)
    sub2 = session.declare_subscriber('identify', identify_listener)
    sub3 = session.declare_subscriber('finish', finish_listener)
    print("Started listening")
    while True:
        time.sleep(1)
        pub1 = session.declare_publisher('rover_state').put(in_mission)
