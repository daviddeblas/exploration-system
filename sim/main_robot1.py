import zenoh
import time
import rospy
import roslaunch
from geometry_msgs.msg import Twist, PoseStamped
from map_generation import map_generation
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import tf

NAME = "rover"

package_name = "limo_gazebo_sim"
launch_file_name = "explore.launch"
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_file_path = roslaunch.rlutil.resolve_launch_arguments(
    [package_name, launch_file_name])[0]
launch_exploration = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
start_exploration = False
exploration_running = False
initial_data = {'x': 0, 'y': 0}
session = zenoh.open()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_limo1', anonymous=False)
rate = rospy.Rate(10)
move = Twist()

last_position = None
last_scan = None

tfBuffer = tf.TransformListener()


def start_listener(sample):
    global exploration_running
    global start_exploration

    if not exploration_running:
        move.angular.z = 1.5708
        pub.publish(move)
        time.sleep(2.0)

    message = sample.payload.decode('utf-8')
    print(message)
    start_exploration = True


def identify_listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)


def finish_listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)
    global launch_exploration
    launch_exploration.shutdown()
    launch_exploration = roslaunch.parent.ROSLaunchParent(
        uuid, [launch_file_path])

    global exploration_running
    exploration_running = False


def odom_callback(data):
    global last_position
    last_position = data.pose.pose.position


def scan_callback(data):
    global last_scan
    last_scan = data


def map_callback(data):
    png = map_generation(data, "map", tfBuffer, True, True)

    # Envoyer l'image par Zenoh
    session.declare_publisher('map_image').put(png.tobytes())


def return_home_listener(sample):
    global initial_data
    global exploration_running
    if exploration_running:
        return
    message = sample.payload.decode('utf-8')
    print(message)
    return_pub = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=10)
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.position.x = initial_data['x']
    msg.pose.position.y = initial_data['y']
    msg.pose.position.z = 0.0
    msg.pose.orientation.w = 1.0

    # Publish the message
    return_pub.publish(msg)


def main():
    global start_exploration
    global exploration_running
    global launch_exploration
    global initial_data
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/limo/scan", LaserScan, scan_callback)
    logger_pub = session.declare_publisher('logger')
    odom_msg = rospy.wait_for_message('/odom', Odometry)
    initial_x = odom_msg.pose.pose.position.x
    initial_y = odom_msg.pose.pose.position.y
    start_sub = session.declare_subscriber('start', start_listener)
    identify_sub = session.declare_subscriber('identify', identify_listener)
    finish_sub = session.declare_subscriber('finish', finish_listener)

    initial_data = {'x': initial_x, 'y': initial_y}

    return_home_sub = session.declare_subscriber(
        'return_home', return_home_listener)
    time.sleep(0.5)
    sub_map = rospy.Subscriber('/limo/map', OccupancyGrid, map_callback)
    print("Started listening")
    while True:
        time.sleep(1)
        if start_exploration and not exploration_running:
            launch_exploration.start()
            start_exploration = False
            exploration_running = True
        pub1 = session.declare_publisher(
            'rover_state').put(exploration_running)
        logger_pub.put(f"{NAME};;position;;{str(last_position)}")
        logger_pub.put(f"{NAME};;scan;;{str(last_scan)}")


if __name__ == "__main__":
    main()
