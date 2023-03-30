import zenoh
import time
import rospy
import roslaunch
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from cv_bridge import CvBridge
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

bridge = CvBridge()

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
        "map", "base_footprint", rospy.Time())

    robot_pos_x = int((trans[0] - cropped_origin_x) /
                      cropped_info.info.resolution)
    robot_pos_y = int((trans[1] - cropped_origin_y) /
                      cropped_info.info.resolution)

    # Dessine un rectangle au niveau de la position du limo
    cv2.rectangle(map_image, (robot_pos_x-2, robot_pos_y-2),
                  (robot_pos_x+2, robot_pos_y+2), (255, 0, 0, 255), thickness=-1)

    # Encoder l'image en PNG
    ret, png = cv2.imencode('.png', map_image)

    # Envoyer l'image par Zenoh
    session.declare_publisher('map_image').put(png.tobytes())


def return_home_listener(sample):
    global initial_data
    global exploration_running
    if exploration_running:
        global launch_exploration
        launch_exploration.shutdown()
        launch_exploration = roslaunch.parent.ROSLaunchParent(
            uuid, [launch_file_path])
        exploration_running = False
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
    sub_map = rospy.Subscriber('/map', OccupancyGrid, map_callback)
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
