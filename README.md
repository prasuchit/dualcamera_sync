# dualcamera_sync
Author and owner: Prasanth Sengadu Suresh.

This package uses a builtin ROS function - [ApproximateTimeSynchronizer](http://docs.ros.org/en/indigo/api/message_filters/html/python/), to save time synchronized messages from two topics into a rosbag file.

It is built for two camera sync, however, it's quite easy to extend or modify this to any number of topics/datatypes, as long as the ros topic contains a timestamp.

## Usage:

    cd catkin_ws/src
 
    git clone https://github.com/prasuchit/dualcamera_sync.git

    cd ..

    catkin_make

    rosrun dualcamera_sync sync_script.py --topic1 TOPIC1 --topic2 TOPIC2 --max_offset 0.5

sync_script.py, located inside dualcamera_sync/scripts folder just subscribes to the topics, checks for timestamp within 0.5 seconds and saves them into a rosbag inside the callback function.
