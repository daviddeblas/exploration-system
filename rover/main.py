import cv2
import numpy as np
import roslaunch
import rospy
import subprocess
from threading import Thread
import time
import tf
import zenoh, math

from cognifly_movement import MoveCognifly
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

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
exploration_running = False
initial_data = {'x': 0, 'y': 0}

session = zenoh.open()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_limo1', anonymous=False)
rate = rospy.Rate(10)
move = Twist()

last_position = None
last_scan = None
activate_p2p = False

bridge = CvBridge()

tfBuffer = tf.TransformListener()

cognifly = MoveCognifly()

def start_listener(sample):
    global start_exploration
    mission_thread = Thread(target=cognifly.start_mission)
    mission_thread.start()

    message = sample.payload.decode('utf-8')
    print(message)
    start_exploration = True

def identify_rover():
    subprocess.call(['aplay', '-q', '--device', 'hw:2,0', '/inf3995_ws/src/beep.wav'])

def identify_listener(sample):
    global cognifly
    global session
    cognifly.identify_cognifly()
    message = sample.payload.decode('utf-8')
    print(message)
    identify_rover()
    
def finish_listener(sample):
    message = sample.payload.decode('utf-8')
    print(message)
    global launch_exploration
    global exploration_running
    finish_thread = Thread(target=cognifly.finish_mission)
    finish_thread.start()

    launch_exploration.shutdown()
    launch_exploration = roslaunch.parent.ROSLaunchParent(
        uuid, [launch_file_path])

    exploration_running = False

def publish_cognifly_odom():
    x, y, z = tuple(value/PUT_IN_CM for value in cognifly.cf.get_position())
    odom = Odometry()

    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'simple_quad_odom_global'
    odom.child_frame_id = 'simple_quad_base_link_global'

    odom.pose.pose = Pose(Point(x=x, y=y, z=z), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

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

def odom_callback(data):
    global last_position
    last_position = data.pose.pose.position
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
    rover_distance = ( math.sqrt(last_position.x**2 + last_position.y**2) ) * PUT_IN_CM
    if (cognifly.distance_calculation() > rover_distance):
        cognifly.identify_cognifly()
    else :
        identify_rover()

def p2p_trigger(sample):
    global activate_p2p
    activate_p2p = not activate_p2p

def return_home_listener(sample):
    global initial_data
    global exploration_running

    cognifly.finish_mission()
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
    global cognifly
    global activate_p2p

    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    logger_pub = session.declare_publisher('logger')

    odom_msg = rospy.wait_for_message('/odom', Odometry)
    initial_x = odom_msg.pose.pose.position.x
    initial_y = odom_msg.pose.pose.position.y

    start_sub = session.declare_subscriber('start', start_listener)
    identify_sub = session.declare_subscriber('identify', identify_listener)
    finish_sub = session.declare_subscriber('finish', finish_listener)
    p2p_sub = session.declare_subscriber('p2p', p2p_trigger)
    return_home_sub = session.declare_subscriber(
        'return_home', return_home_listener)

    initial_data = {'x': initial_x, 'y': initial_y}

    sub_map = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    print("Started listening")
    
    drone_state_pub = session.declare_publisher('drone_state')
    rover_state_pub= session.declare_publisher('rover_state')
    while True:
        time.sleep(1)
        if start_exploration and not exploration_running:
            launch_exploration.start()
            start_exploration = False
            exploration_running = True
        rover_state_pub.put(exploration_running)
        if cognifly.is_crashed():
            drone_state_pub.put("Crashed")
        else:
            drone_state_pub.put(exploration_running)
        logger_pub.put(f"{NAME};;position;;{str(last_position)}")
        logger_pub.put(f"{NAME};;scan;;{str(last_scan)}")
        if (activate_p2p): farthest_robot_trigger()

if __name__ == "__main__":
    main()
