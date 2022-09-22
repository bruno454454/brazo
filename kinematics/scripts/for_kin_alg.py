#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from sympy import *
from geometry_msgs.msg import Point
L1=0.4
L2=0.4
t1=Symbol('t1')
t2=Symbol('t2')
def publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    point_pub = rospy.Publisher('position', Point, queue_size=10)
    rospy.init_node('for_kin_alg', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    pos=Point()
    joints=JointState()
    hd=Header()
    seq=0
    joints.name=["joint1","joint2"]
    angles=[0,0]
    x=L2*cos(t2)*cos(t1)
    y=L2*cos(t2)*sin(t1)
    z=L1-L2*sin(t2)
    while not rospy.is_shutdown():
	hd.seq=seq
        hd.stamp=rospy.Time.now()
	joints.header=hd
	angles[0]+=0.2
	angles[1]+=0.3
	pos.x=x.subs([(t1,angles[0]),(t2,angles[1])])
	pos.y=y.subs([(t1,angles[0]),(t2,angles[1])])
	pos.z=z.subs([(t1,angles[0]),(t2,angles[0])])

	joints.position=angles
	pub.publish(joints)
        point_pub.publish(pos)
	seq+=1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

