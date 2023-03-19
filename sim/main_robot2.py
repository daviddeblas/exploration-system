import zenoh
import time
import rospy
from geometry_msgs.msg import Twist
import threading
from constants import TIME_TO_TURN_90, TIME_TO_TURN_145, END_LINE_TIME, REPOSITION_TIME, CROSS_THE_MAP_TIME, NUMBER_OF_LINE

pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_drone', anonymous=False)
rate = rospy.Rate(10)
move = Twist()

exploration_running = False
stop_event = threading.Event()
line_counter = 0


def rotation_left():
    move.angular.z = 1.0
    pub.publish(move)
    rospy.sleep(TIME_TO_TURN_90)
    move.angular.z = 0.0
    pub.publish(move)


def rotation_right():
    move.angular.z = -1.0
    pub.publish(move)
    rospy.sleep(TIME_TO_TURN_90)
    move.angular.z = 0.0
    pub.publish(move)


def forward():
    rospy.sleep(0.5)
    move.linear.x = 1.0
    pub.publish(move)
    rospy.sleep(END_LINE_TIME)
    move.linear.x = 0.0
    pub.publish(move)


def back_to_base():
    move.angular.z = -1.0
    pub.publish(move)
    rospy.sleep(TIME_TO_TURN_145)
    move.angular.z = 0.0
    pub.publish(move)

    rospy.sleep(0.5)
    move.linear.x = 1.0
    pub.publish(move)
    rospy.sleep(CROSS_THE_MAP_TIME)
    move.linear.x = 0.0
    pub.publish(move)

    move.angular.z = 1.0
    pub.publish(move)
    rospy.sleep(TIME_TO_TURN_145)
    move.angular.z = 0.0
    pub.publish(move)


def drone_movement(line_number, start_line):
    global line_counter, exploration_running

    for i in range(start_line, line_number):
        if stop_event.is_set():
            break

        forward()

        if (i != line_number - 1):
            if (line_counter % 2 == 0):
                rotation_left()
            else:
                rotation_right()

            move.linear.x = 0.2
            pub.publish(move)
            rospy.sleep(REPOSITION_TIME)
            move.linear.x = 0.0
            pub.publish(move)

            if (line_counter % 2 == 0):
                rotation_left()
            else:
                rotation_right()

            line_counter = i + 1
        else:
            back_to_base()
            line_counter = 0
            exploration_running = False


def stop_drone_movement():
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)


def start_listener(sample):
    global drone_thread, exploration_running, line_counter
    message = sample.payload.decode('utf-8')
    print(message)
    exploration_running = True
    stop_event.clear()

    drone_thread = threading.Thread(
        target=drone_movement, args=(NUMBER_OF_LINE, line_counter,))
    drone_thread.start()


def identify_listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)


def finish_listener(sample):
    global exploration_running, drone_thread
    exploration_running = False
    # Ces 2 commandes laissent le drone finir sa manoeuvre avant de le stopper
    stop_event.set()
    drone_thread.join()

    message = sample.payload.decode('utf-8')
    print(message)
    stop_drone_movement()


def main():
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
        pub1 = session.declare_publisher('drone_state').put(exploration_running)


if __name__ == "__main__":
    main()
