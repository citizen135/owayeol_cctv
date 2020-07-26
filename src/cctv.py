#!/usr/bin/env python

import rospy
import move_base
from std_msgs.msg import String,UInt16
from darknet_ros_msgs.msg import BoundingBoxes

servox=90
servoy=90

def ALERT(data):
    global servox
    global servoy
    
    for st in data.bounding_boxes:           
        print(st.Class)
        if st.Class=="cell phone":
            midx=(st.xmin+st.xmax)/2
            if midx<300:
                servox=servox+1
            elif midx>340:
                servox=servox-1
            midy=(st.ymin+st.ymax)/2
            if midy<200:
                servoy=servoy-1
            elif midy>280:
                servoy=servoy+1
            servox_pub.publish(servox)
            servoy_pub.publish(servoy)
            

if __name__ == '__main__':
    rospy.init_node('owayeol_cctv')
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,ALERT)
    servox_pub = rospy.Publisher("/servo_x",UInt16, queue_size=1)
    servoy_pub = rospy.Publisher("/servo_y",UInt16, queue_size=1)
    servox_pub.publish(servox)
    servoy_pub.publish(servoy)

    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            pass
