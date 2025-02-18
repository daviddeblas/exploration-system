
import cv2
import numpy as np
import roslaunch
import rospy
import subprocess
from threading import Thread
import time
import tf
import zenoh
import math

from cognifly_movement import MoveCognifly
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from limo_base.msg import LimoStatus
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

NAME = "rover"
PUT_IN_CM = 100

package_name = "limo_bringup"
launch_file_name = "explore.launch"

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_file_path = roslaunch.rlutil.resolve_launch_arguments(
    [package_name, launch_file_name])[0]
launch_exploration = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
start_exploration = False
exploration_running_rover = False
initial_data = {'x': 0, 'y': 0}

session = zenoh.open()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_limo1', anonymous=False)
rate = rospy.Rate(10)
move = Twist()

last_position = None
distance_traveled = 0
cognifly_last_position = None
cognifly_distance_traveled = 0
last_scan = None
current_battery_rover = None
activate_p2p = False

bridge = CvBridge()
tfBuffer = tf.TransformListener()

cognifly = MoveCognifly()
exploration_running_drone = False


def start_listener_rover(sample):
    global start_exploration
    mission_thread = Thread(target=cognifly.start_mission)
    mission_thread.start()

    message = sample.payload.decode('utf-8')
    print(message)
    start_exploration = True


def start_listener_drone(sample):
    global exploration_running_drone

    message = sample.payload.decode('utf-8')
    print(message)
    exploration_running_drone = True
    mission_thread = Thread(target=cognifly.start_mission)
    mission_thread.start()


def identify_rover():
    subprocess.call(['aplay', '-q', '--device', 'hw:2,0',
                    '/inf3995_ws/src/beep.wav'])


def identify_listener(sample):
    global cognifly
    global session
    cognifly.identify_cognifly()
    message = sample.payload.decode('utf-8')
    print(message)
    identify_rover()


def finish_listener(sample):
    global launch_exploration
    global exploration_running_rover
    global exploration_running_drone
    global cognifly
    message = sample.payload.decode('utf-8')
    print(message)
    global exploration_running
    finish_thread = Thread(target=cognifly.finish_mission)
    finish_thread.start()

    launch_exploration.shutdown()
    launch_exploration = roslaunch.parent.ROSLaunchParent(
        uuid, [launch_file_path])
    exploration_running_rover = False
    exploration_running_drone = False

    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

    # Cancel la mission en cours
    cancel_msg = GoalID()
    cancel_msg.stamp = rospy.Time.now()

    cancel_pub.publish(cancel_msg)

    rospy.sleep(0.5)


def publish_cognifly_odom():
    global cognifly_last_position
    global cognifly_distance_traveled

    x, y, z = tuple(value/PUT_IN_CM for value in cognifly.cf.get_position())
    y = -y
    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'simple_quad_odom_global'
    odom.child_frame_id = 'simple_quad_base_link_global'

    odom.pose.pose = Pose(Point(x=x, y=y, z=z),
                          Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

    odom_pub = rospy.Publisher('/cognifly/odom', Odometry, queue_size=10)

    odom.header.stamp = rospy.Time.now()
    odom_pub.publish(odom)

    # Publier la transformée entre l'odom et le base_link du cognifly
    tf_broadcaster = tf.TransformBroadcaster()
    tf_broadcaster.sendTransform(
        (x, y, z),
        (0.0, 0.0, 0.0, 1.0),
        rospy.Time.now(),
        'simple_quad_base_link_global',
        'simple_quad_odom_global'
    )

    position = odom.pose.pose.position
    if cognifly_last_position is not None:
        cognifly_distance_traveled += math.sqrt(
            (cognifly_last_position.x - position.x) ** 2 +
            (cognifly_last_position.y - position.y) ** 2)
    cognifly_last_position = position


def odom_callback(data):
    global last_position
    global distance_traveled
    position = data.pose.pose.position
    if last_position is not None:
        distance_traveled += math.sqrt(
            (last_position.x - position.x) ** 2 +
            (last_position.y - position.y) ** 2)
    last_position = position
    publish_cognifly_odom()


def scan_callback(data):
    global last_scan
    last_scan = data


def map_callback(data):
    # Convertir le message OccupancyGrid vers une image RGBA OpenCV
    map_array = np.array(data.data, dtype=np.int8).reshape(
        (data.info.height, data.info.width))

    # Enlever tous les lignes et colonnes contenant seulement des -1
    non_free_rows, non_free_cols = np.nonzero(map_array != -1)
    cropped_array = map_array[
        non_free_rows.min(): non_free_rows.max() + 1,
        non_free_cols.min(): non_free_cols.max() + 1,
    ]

    # Ajuster la carte pour que les coordonnées soient respectées
    cropped_origin_x = data.info.origin.position.x + \
        non_free_cols.min() * data.info.resolution
    cropped_origin_y = data.info.origin.position.y + \
        non_free_rows.min() * data.info.resolution
    cropped_height, cropped_width = cropped_array.shape
    cropped_info = OccupancyGrid(
        header=data.header,
        info=data.info,
        data=cropped_array.flatten().tolist(),
    )
    cropped_info.info.width = cropped_width
    cropped_info.info.height = cropped_height
    cropped_info.info.origin.position.x = cropped_origin_x
    cropped_info.info.origin.position.y = cropped_origin_y

    # Créer l'image de couleur
    map_image = np.zeros((cropped_height, cropped_width, 4), dtype=np.uint8)

    # Mapper les valeurs de OccupancyGrid vers les valeurs des images
    map_image[cropped_array == -1] = [128, 128, 128, 0]   # Inconnues
    map_image[(cropped_array >= 0) & (cropped_array <= 50)] = [
        255, 255, 255, 255]    # Espace ouvert
    map_image[(cropped_array > 50) & (cropped_array <= 100)] = [
        0, 0, 0, 255]    # Espace Occupé

    trans, rot = tfBuffer.lookupTransform(
        "map", "base_link", rospy.Time())

    robot_pos_x = int((trans[0] - cropped_origin_x) /
                      cropped_info.info.resolution)
    robot_pos_y = int((trans[1] - cropped_origin_y) /
                      cropped_info.info.resolution)

    # Dessine un rectangle au niveau de la position du limo
    cv2.rectangle(map_image, (robot_pos_x-2, robot_pos_y-2),
                  (robot_pos_x+2, robot_pos_y+2), (255, 0, 0, 255), thickness=-1)

    # Ajouter la position du cognifly de la même manière
    trans_cognifly, rot_cognifly = tfBuffer.lookupTransform(
        "map", "simple_quad_base_link_global", rospy.Time())

    cognifly_pos_x = int((trans_cognifly[0] - cropped_origin_x) /
                         cropped_info.info.resolution)
    cognifly_pos_y = int((trans_cognifly[1] - cropped_origin_y) /
                         cropped_info.info.resolution)

    # Dessine un rectangle au niveau de la position du limo
    cv2.rectangle(map_image, (cognifly_pos_x-2, cognifly_pos_y-2),
                  (cognifly_pos_x+2, cognifly_pos_y+2), (0, 255, 0, 255), thickness=-1)

    # Encoder l'image en PNG
    ret, png = cv2.imencode('.png', map_image)

    # Envoyer l'image par Zenoh
    session.declare_publisher('map_image').put(png.tobytes())


def farthest_robot_trigger():
    global cognifly
    global last_position
    rover_distance = (math.sqrt(last_position.x**2 +
                      last_position.y**2)) * PUT_IN_CM
    if (cognifly.distance_calculation() > rover_distance):
        cognifly.identify_cognifly()
    else:
        identify_rover()


def p2p_trigger(sample):
    global activate_p2p
    activate_p2p = not activate_p2p


def battery_rover_callback(data):
    global current_battery_rover
    voltage = float(data.battery_voltage)
    current_battery_rover = int((voltage - 8.3) / (12.6 - 8.3) * 100)


def return_home():
    global initial_data
    global exploration_running_rover

    if exploration_running_rover:
        global launch_exploration
        launch_exploration.shutdown()
        launch_exploration = roslaunch.parent.ROSLaunchParent(
            uuid, [launch_file_path])
        exploration_running_rover = False

    # Cancel la mission en cours
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

    cancel_msg = GoalID()
    cancel_msg.stamp = rospy.Time.now()

    cancel_pub.publish(cancel_msg)

    rospy.sleep(0.5)
    return_pub = rospy.Publisher(
        '/move_base/goal', MoveBaseActionGoal, queue_size=10)
    msg = MoveBaseActionGoal()
    msg.header.stamp = rospy.Time.now()
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.position.x = initial_data['x']
    msg.goal.target_pose.pose.position.y = initial_data['y']
    msg.goal.target_pose.pose.orientation.w = 1.0

    # Publish the message
    return_pub.publish(msg)


def return_home_rover_listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)
    return_home()


def return_home_listener(sample):
    global exploration_running_drone
    global cognifly
    message = sample.payload.decode('utf-8')
    print(message)
    exploration_running_drone = False
    finish_thread = Thread(target=cognifly.finish_mission)
    finish_thread.start()
    return_home()


def main():
    global start_exploration
    global exploration_running_rover
    global launch_exploration
    global initial_data
    global cognifly
    global exploration_running_drone
    global activate_p2p
    global distance_traveled
    global cognifly_distance_traveled

    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/limo_status", LimoStatus, battery_rover_callback)

    logger_pub = session.declare_publisher('logger')

    odom_msg = rospy.wait_for_message('/odom', Odometry)
    initial_x = odom_msg.pose.pose.position.x
    initial_y = odom_msg.pose.pose.position.y

    start_sub = session.declare_subscriber('start_rover', start_listener_rover)
    start_sub2 = session.declare_subscriber(
        'start_drone', start_listener_drone)
    identify_sub = session.declare_subscriber('identify', identify_listener)

    finish_sub = session.declare_subscriber('finish', finish_listener)
    p2p_sub = session.declare_subscriber('p2p', p2p_trigger)
    return_home_sub = session.declare_subscriber(
        'return_home', return_home_listener)
    return_home_sub_rover = session.declare_subscriber(
        'return_home_rover', return_home_rover_listener)

    initial_data = {'x': initial_x, 'y': initial_y}

    sub_map = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    print("Started listening")

    drone_state_pub = session.declare_publisher('drone_state')
    rover_state_pub = session.declare_publisher('rover_state')
    pub_rover_battery = session.declare_publisher('rover_battery')
    pub_drone_battery = session.declare_publisher('drone_battery')
    distance_traveled_pub = session.declare_publisher(
        'rover_distance_traveled')
    cognifly_distance_traveled_pub = session.declare_publisher(
        'drone_distance_traveled')
    while True:
        time.sleep(1)
        if start_exploration and not exploration_running_rover:
            launch_exploration.start()
            start_exploration = False
            exploration_running_rover = True
        rover_state_pub.put(exploration_running_rover)
        distance_traveled_pub.put(distance_traveled)
        cognifly_distance_traveled_pub.put(cognifly_distance_traveled)
        if cognifly.is_crashed():
            drone_state_pub.put("Crashed")
        else:
            drone_state_pub.put(exploration_running_drone)
        logger_pub.put(f"{NAME};;position;;{str(last_position)}")
        logger_pub.put(f"{NAME};;scan;;{str(last_scan)}")
        pub_rover_battery.put(current_battery_rover)
        pub_drone_battery.put(cognifly.get_battery())

        if (activate_p2p):
            farthest_robot_trigger()


if __name__ == "__main__":
    main()
