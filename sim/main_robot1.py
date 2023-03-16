import zenoh
import time
import rospy
import roslaunch
import cv2
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Odometry
import tf

package_name = "limo_gazebo_sim"
launch_file_name = "explore.launch"
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_file_path = roslaunch.rlutil.resolve_launch_arguments([package_name, launch_file_name])[0]
launch_exploration = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
start_exploration = False
exploration_running = False
initial_data = {'x': 0, 'y': 0}
session = zenoh.open()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_limo1', anonymous=False)
rate = rospy.Rate(10)
move = Twist()

bridge = CvBridge()

tfBuffer = tf.TransformListener()

def start_listener(sample):
    global exploration_running
    global start_exploration
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
    launch_exploration = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
    
    global exploration_running
    exploration_running = False

def map_callback(data):
    # Convert the OccupancyGrid message to a grayscale OpenCV image
    map_array = np.array(data.data, dtype=np.int8).reshape((data.info.height, data.info.width))

    # Crop the map to remove all lines and columns containing only -1
    non_free_rows, non_free_cols = np.nonzero(map_array != -1)
    cropped_array = map_array[
        non_free_rows.min(): non_free_rows.max() + 1,
        non_free_cols.min(): non_free_cols.max() + 1,
    ]

    # Adjust the map metadata to reflect the cropping
    cropped_origin_x = data.info.origin.position.x + non_free_cols.min() * data.info.resolution
    cropped_origin_y = data.info.origin.position.y + non_free_rows.min() * data.info.resolution
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

    # Create a color image
    map_image = np.zeros((cropped_height, cropped_width, 4), dtype=np.uint8)

    # Map the occupancy grid values to grayscale values for visual purposes
    map_image[cropped_array == -1] = [128, 128, 128, 0]   # Unknown
    map_image[(cropped_array >= 0) & (cropped_array <= 50)] = [255, 255, 255, 255]    # Free
    map_image[(cropped_array > 50) & (cropped_array <= 100)] = [0, 0, 0, 255]    # Occupied

    trans, rot = tfBuffer.lookupTransform("map", "base_footprint", rospy.Time())
    
    robot_pos_x = int((trans[0] - cropped_origin_x) / cropped_info.info.resolution)
    robot_pos_y = int((trans[1] - cropped_origin_y) / cropped_info.info.resolution)

    # Draw a rectangle on the map to represent the robot
    cv2.rectangle(map_image, (robot_pos_x-2, robot_pos_y-2), (robot_pos_x+2, robot_pos_y+2), (255, 0, 0, 255), thickness=-1)
    
    arrow_len = 10
    arrow_start = (robot_pos_x, robot_pos_y)
    arrow_end = (int(robot_pos_x + arrow_len * np.cos(rot[2])),
                 int(robot_pos_y + arrow_len * np.sin(rot[2])))
    cv2.arrowedLine(map_image, arrow_start, arrow_end, (0, 255, 0, 255), thickness=1, tipLength=0.5)

    # Encode the image as PNG
    ret, png = cv2.imencode('.png', map_image)

    # Publish the PNG image on a Zenoh topic
    session.declare_publisher('map_image').put(png.tobytes())

def return_home_listener(sample):
    global initial_data
    global exploration_running
    if exploration_running:
        return
    message = sample.payload.decode('utf-8')
    print(message)
    return_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
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
    odom_msg = rospy.wait_for_message('/odom', Odometry)
    initial_x = odom_msg.pose.pose.position.x
    initial_y = odom_msg.pose.pose.position.y
    start_sub = session.declare_subscriber('start', start_listener)
    identify_sub = session.declare_subscriber('identify', identify_listener)
    finish_sub = session.declare_subscriber('finish', finish_listener)
    
    initial_data = {'x': initial_x, 'y': initial_y}
    
    return_home_sub= session.declare_subscriber('return_home', return_home_listener)
    time.sleep(0.5)
    sub_map = rospy.Subscriber('/map', OccupancyGrid, map_callback)
    print("Started listening")
    while True:
        time.sleep(1)
        if start_exploration and not exploration_running:
            launch_exploration.start()
            start_exploration = False
            exploration_running = True
        pub1 = session.declare_publisher('rover_state').put(exploration_running)

if __name__ == "__main__":
    main()
