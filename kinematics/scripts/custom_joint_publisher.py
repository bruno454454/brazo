#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
def publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('custom_joint_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    joints=JointState()
    hd=Header()
    seq=0
    joints.name=["joint_1","joint_2","joint_3","joint_4"]
    angles=[0,0,0,0]
    while not rospy.is_shutdown():
	hd.seq=seq
        hd.stamp=rospy.Time.now()
	joints.header=hd
	angles[0]+=1
	angles[1]+=0.2
	angles[2]+=0.5
	angles[3]+=1
	joints.position=angles
	pub.publish(joints)
	seq+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

