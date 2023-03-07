import zenoh
import time
import rospy
import roslaunch
from geometry_msgs.msg import Twist

package_name = "cartographer_ros"
launch_file_name = "explore.launch"
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch_file_path = roslaunch.rlutil.resolve_launch_arguments([package_name, launch_file_name])[0]
launch_exploration = roslaunch.parent.ROSLaunchParent(uuid, [launch_file_path])
start_exploration = False
exploration_running = False

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.init_node('movement_limo1', anonymous=False)
rate = rospy.Rate(10)
move = Twist()

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

def main():
    global start_exploration
    global exploration_running
    global launch_exploration
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
        if start_exploration and not exploration_running:
            launch_exploration.start()
            start_exploration = False
            exploration_running = True
        pub1 = session.declare_publisher('rover_state').put(exploration_running)

if __name__ == "__main__":
    main()
