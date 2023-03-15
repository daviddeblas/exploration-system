import zenoh
import time
import rospy
import roslaunch
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
import tf

package_name = "cartographer_ros"
launch_file_name = "explore.launch"
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_file_path = roslaunch.rlutil.resolve_launch_arguments([package_name, launch_file_name])[0]
launch_exploration = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
start_exploration = False
exploration_running = False

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

    # Create a color image
    map_image = np.zeros((data.info.height, data.info.width, 3), dtype=np.uint8)

    # Map the occupancy grid values to grayscale values for visual purposes
    map_image[map_array == -1] = [128, 128, 128]   # Unknown
    map_image[(map_array >= 0) & (map_array <= 50)] = [255, 255, 255]    # Free
    map_image[(map_array > 50) & (map_array <= 100)] = [0, 0, 0]    # Occupied

    trans, rot = tfBuffer.lookupTransform("map", "base_footprint", rospy.Time())
    
    robot_pos_x = int((trans[0] - data.info.origin.position.x) / data.info.resolution)
    robot_pos_y = int((trans[1] - data.info.origin.position.y) / data.info.resolution)

    # Draw a rectangle on the map to represent the robot
    cv2.rectangle(map_image, (robot_pos_x-2, robot_pos_y-2), (robot_pos_x+2, robot_pos_y+2), (255, 0, 0), thickness=-1)
    
    arrow_len = 10
    arrow_start = (robot_pos_x, robot_pos_y)
    arrow_end = (int(robot_pos_x + arrow_len * np.cos(rot[2])),
                 int(robot_pos_y + arrow_len * np.sin(rot[2])))
    cv2.arrowedLine(map_image, arrow_start, arrow_end, (0, 255, 0), thickness=1, tipLength=0.5)


    # Encode the image as JPEG
    ret, jpeg = cv2.imencode('.jpg', map_image)

    # Publish the JPEG image on a Zenoh topic
    session.declare_publisher('map_image').put(jpeg.tobytes())

def main():
    global start_exploration
    global exploration_running
    global launch_exploration
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    sub1 = session.declare_subscriber('start', start_listener)
    sub2 = session.declare_subscriber('identify', identify_listener)
    sub3 = session.declare_subscriber('finish', finish_listener)
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
