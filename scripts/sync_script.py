#!/usr/bin/env python

import os
import sys
import cv2
import time
import rospy
import random
import pprint
import argparse
import image_geometry
import message_filters
from copy import copy
import numpy as np
from itertools import chain
from rospy.core import rospyinfo
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import rosbag
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import datetime
import time
import rospkg

data1 = None
data2 = None
updated = False
pub1, pub2, bag = None, None, None
topic1 = None
topic2 = None

def callback(msg1, msg2):
    global data1, data2, updated, bag, pub1, pub2, topic1, topic2
    # pub1.publish(msg1)
    # pub2.publish(msg2)
    print("Storing images...")
    bag.write(topic1, msg1)
    bag.write(topic2, msg2)


def main(args):
    global updated, data1, data2, bag, pub1, pub2, topic1, topic2
    parser = argparse.ArgumentParser(description='Record a bagfile with time synced topics')
    parser.add_argument('--topic1', nargs=1, help='input topic1 name')
    parser.add_argument('--topic2', nargs=1, help='input topic2 name')
    parser.add_argument('--max_offset', nargs=1, help='max time offset (sec) to correct.', default=[0.5], type=float)
    args = parser.parse_args()
    dt = time.strftime("%Y%m%d-%H%M%S")
    rospack = rospkg.RosPack()
    path = rospack.get_path('dummy_pkg') 
    bag = rosbag.Bag(path + '/bags/' + dt + '.bag', 'w')
    q = 10

    rospy.init_node('sync_sub', anonymous=True)

    topic1 = args.topic1[0]
    topic2 = args.topic2[0]

    sub_rgb1 = message_filters.Subscriber(args.topic1[0], Image, queue_size=q)

    sub_rgb2 = message_filters.Subscriber(args.topic2[0], Image, queue_size=q)

    # pub1 = rospy.Publisher('image1', Image, queue_size=q)
    # pub2 = rospy.Publisher('image2', Image, queue_size=q)


    tss = message_filters.ApproximateTimeSynchronizer([sub_rgb1, sub_rgb2], queue_size = q, slop = args.max_offset[0])
    try:
        tss.registerCallback(callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Interrupted!")
        bag.close()
        rospy.sleep(0.1)
        rospy.signal_shutdown()
    finally:
        bag.close()


if __name__ == "__main__":
  main(sys.argv[1:])