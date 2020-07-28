#!/usr/bin/env python

import rospy
import move_base
from std_msgs.msg import String,Int32,Header
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import JointState
servox=0
servoy=0

def ALERT(data):
    global servox
    global servoy
    
    for st in data.bounding_boxes:           
        print(st.Class)
        if (st.Class=="person") and (st.probability>0.5):
            midx=(st.xmin+st.xmax)/2
            oh=st.xmax-st.xmin
            if midx<(320-oh):
                servox=servox+0.1
            elif midx>(320+oh):
                servox=servox-0.1
            midy=(st.ymax+st.ymin)/2
            yeol=st.ymax-st.ymin
            if midy<(240-yeol):
                servoy=servoy-0.1
            elif midy>(240+yeol):
                servoy=servoy+0.1
            
def talker():
    global servox
    global servoy

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(1) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['joint0', 'joint1']
    hello_str.position = [servox, servoy]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    rate.sleep()

if __name__ == '__main__':
    #rospy.init_node('owayeol_cctv')
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,ALERT)
    # servox_pub = rospy.Publisher("/servo_x",UInt16, queue_size=1)
    # servoy_pub = rospy.Publisher("/servo_y",UInt16, queue_size=1)
    # servox_pub.publish(servox)
    # servoy_pub.publish(servoy)

    while not rospy.is_shutdown():
        try:
            talker()
        except rospy.ROSInterruptException:
            pass
