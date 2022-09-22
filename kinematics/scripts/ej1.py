#!/usr/bin/env python
import rospy
from sympy import*
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

d1=Symbol('d1')
d2=Symbol('d2')
d3=Symbol('d3')


def dh_matrix (t,d,a,aph):

	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],[sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],[0,sin(aph), cos(aph), d],[0, 0, 0, 1]])
	return T



T01=dh_matrix(0,0,0,pi/2)
T11p=dh_matrix(0,d1,0.3,0)
T1p2=dh_matrix(pi/2,0,0,pi/2)
T22p=dh_matrix(0,d2,0.4,0)
T2p3=dh_matrix(pi/2,0,0,pi/2)
T33p=dh_matrix(pi/2,d3,0.25,0)
T3p4=dh_matrix(-pi/2,0,0,0)



T01p=T01*T11p
T02=T01p*T1p2
T02p=T02*T22p
T03=T02p*T2p3
T03p=T03*T33p
T04=T03p*T3p4


pub = rospy.Publisher('joint_states', JointState, queue_size=1)
pos = rospy.Publisher('position', Point, queue_size=1)
position = Point()
header = Header()
seq = 0
header.frame_id=''
joint_msg=JointState()
joint_msg.name = ['joint1','joint2','joint3']
angles=[0.0,0.0,0.0]
def callback(data):
    angles[0]=data.x
    angles[1]=data.y
    angles[2]=data.z
    global seq
    seq=seq+1
    header.seq = seq
    header.stamp = rospy.get_rostime()
    joint_msg.header=header
    joint_msg.position=angles
    pub.publish(joint_msg)
    T04n=T04.subs([(d1,data.x),(d2,data.y),(d3,data.z)])
    position.x= T04n[0,3]
    position.y= T04n[1,3]
    position.z= T04n[2,3]
    pos.publish(position)
    print(angles)
def listener():

   
    rospy.init_node('ej1', anonymous=True)
   
    rospy.Subscriber("angles", Point, callback)
    print('px=', T04[0,3])
    print('py=', T04[1,3])
    print('pz=', T04[2,3])

    rospy.spin() #keep alive
if __name__ == '__main__':
    listener()
