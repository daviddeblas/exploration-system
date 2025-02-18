import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

def callback(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow('Video Feed', frame)
    cv2.waitKey(1)
    rospy.loginfo('Image feed received!')

def listener():
    rospy.init_node('vid_rec')

    rospy.Subscriber('/orb_slam2_mono/debug_image', Image, callback)
    rospy.spin()
    
if __name__ == '__main__':
    listener()
