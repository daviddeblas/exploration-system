import zenoh
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
from constants import TIME_TO_TURN_90, TIME_TO_TURN_135, END_LINE_TIME, REPOSITION_TIME, CROSS_THE_MAP_TIME, NUMBER_OF_LINE
import math

NAME = "drone"

pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_drone', anonymous=False)
rate = rospy.Rate(10)
move = Twist()
session = zenoh.open()

exploration_running = False
stop_event = threading.Event()
line_counter = 0

last_position = None


def rotation_left():
    rospy.sleep(1.5)
    move.angular.z = 1.0
    pub.publish(move)
    rospy.sleep(TIME_TO_TURN_90)
    move.angular.z = 0.0
    pub.publish(move)


def rotation_right():
    rospy.sleep(1.5)
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


# def back_to_base():
#     global line_counter

#     if (line_counter % 2 == 0 and line_counter != NUMBER_OF_LINE - 1):
#         rotation_right()
#         for _ in range(line_counter):
#             move.linear.x = 0.2
#             pub.publish(move)
#             rospy.sleep(REPOSITION_TIME)
#             move.linear.x = 0.0
#             pub.publish(move)
#             rospy.sleep(1)
#         rotation_left()
#     else:
#         rotation_time = math.atan(line_counter / 10)
#         cross_the_map_time = END_LINE_TIME + line_counter * 0.035

#         if (line_counter != 10):
#             move.angular.z = 1.0
#             pub.publish(move)
#             rospy.sleep(rotation_time)
#             move.angular.z = 0.0
#             pub.publish(move)

#             rospy.sleep(1.5)
#             move.linear.x = 1.0
#             pub.publish(move)
#             rospy.sleep(cross_the_map_time)
#             move.linear.x = 0.0
#             pub.publish(move)

#             rospy.sleep(1.5)
#             move.angular.z = 1.0
#             pub.publish(move)
#             rospy.sleep(3.14 + rotation_time)
#             move.angular.z = 0.0
#             pub.publish(move)
#         else:
#             move.angular.z = -1.0
#             pub.publish(move)
#             rospy.sleep(TIME_TO_TURN_135)
#             move.angular.z = 0.0
#             pub.publish(move)

#             rospy.sleep(1.5)
#             move.linear.x = 1.0
#             pub.publish(move)
#             rospy.sleep(cross_the_map_time)
#             move.linear.x = 0.0
#             pub.publish(move)

#             rospy.sleep(1.5)
#             move.angular.z = 1.0
#             pub.publish(move)
#             rospy.sleep(TIME_TO_TURN_135)
#             move.angular.z = 0.0
#             pub.publish(move)
#             line_counter = 0


def back_to_base():
    global line_counter
    if (line_counter != 0):
        if (line_counter % 2 == 0 and line_counter != NUMBER_OF_LINE - 1):
            rotate_right_and_move_forward(line_counter)
            rotation_left()
        else:
            go_to_starting_point(line_counter)
    line_counter = 0


def rotate_right_and_move_forward(line_counter):
    rotation_right()
    move_forward(line_counter)


def go_to_starting_point(line_counter):
    rotation_time = math.atan(line_counter / 10)
    cross_the_map_time = END_LINE_TIME + (line_counter-1) * 0.035

    if (line_counter != 10):
        turn_left_and_move_forward(rotation_time, cross_the_map_time)
        replace_to_initial_orientation(rotation_time)
    else:
        turn_left_and_move_forward(TIME_TO_TURN_135, cross_the_map_time)
        replace_to_initial_orientation(TIME_TO_TURN_135)


def move_forward(line_counter):
    for _ in range(line_counter):
        move.linear.x = 0.2
        pub.publish(move)
        rospy.sleep(REPOSITION_TIME)
        move.linear.x = 0.0
        pub.publish(move)
        rospy.sleep(1)


def turn_left_and_move_forward(rotation_time, cross_the_map_time):
    if (line_counter != 10):
        move.angular.z = 1.0
        pub.publish(move)
        rospy.sleep(rotation_time)
        move.angular.z = 0.0
        pub.publish(move)

        rospy.sleep(1.5)
        move.linear.x = 1.0
        pub.publish(move)
        rospy.sleep(cross_the_map_time)
        move.linear.x = 0.0
        pub.publish(move)
    else:
        move.angular.z = -1.0
        pub.publish(move)
        rospy.sleep(TIME_TO_TURN_135)
        move.angular.z = 0.0
        pub.publish(move)

        rospy.sleep(1.5)
        move.linear.x = 1.0
        pub.publish(move)
        rospy.sleep(CROSS_THE_MAP_TIME)
        move.linear.x = 0.0
        pub.publish(move)


def replace_to_initial_orientation(rotation_time):
    global line_counter
    if (line_counter != 10):
        rospy.sleep(1.5)
        move.angular.z = 1.0
        pub.publish(move)
        rospy.sleep(3.14 - rotation_time)
        move.angular.z = 0.0
        pub.publish(move)
    else:
        rospy.sleep(1.5)
        move.angular.z = 1.0
        pub.publish(move)
        rospy.sleep(TIME_TO_TURN_135)
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
            rospy.sleep(1)
            back_to_base()
            # line_counter = 0
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


def odom_callback(data):
    global last_position
    last_position = data.pose.pose.position


def return_home_listener(sample):
    global exploration_running, line_counter
    message = sample.payload.decode('utf-8')
    print(message)
    stop_event.set()
    drone_thread.join()

    back_to_base()
    exploration_running = False


def main():
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)

    sub1 = session.declare_subscriber('start', start_listener)
    sub2 = session.declare_subscriber('identify', identify_listener)
    sub3 = session.declare_subscriber('finish', finish_listener)
    pub_state = session.declare_publisher('drone_state')
    rospy.Subscriber("/robot2/odom", Odometry, odom_callback)
    logger_pub = session.declare_publisher('logger')

    return_home_sub = session.declare_subscriber(
        'return_home', return_home_listener)

    print("Started listening")
    while True:
        time.sleep(1)
        pub_state.put(exploration_running)
        logger_pub.put(f"{NAME};;position;;{str(last_position)}")


if __name__ == "__main__":
    main()
