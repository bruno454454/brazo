#!/usr/bin/env python
import rospy
from sympy import*
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

d1=Symbol('d1')
t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')
t4=Symbol('t4')
t5=Symbol('t5')


def dh_matrix (t,d,a,aph):

	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],[sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],[0,sin(aph), cos(aph), d],[0, 0, 0, 1]])
	return T



T01=dh_matrix(pi/2,0,0,pi/2)
T11p=dh_matrix(pi/2,d1,0.05,0)
T1p2=dh_matrix(pi/2,0,0,pi/2)
T23=dh_matrix(t1+pi/2,0.6,0,pi/2)
T33p=dh_matrix(t2,0,0.2,0)
T3p4=dh_matrix(-pi/2,0,-0.2,pi/2)
T45=dh_matrix(t3,-0.4,0,pi/2)
T55p=dh_matrix(t4-pi/2,0,0.1,0)
T5p6=dh_matrix(-pi/2,0,0,pi)
T66p=dh_matrix(t5-pi/2,0,0.2,0)
T6p7=dh_matrix(pi/2,0,0,0)



T01p=T01*T11p
T02=T01p*T1p2
T03=T02*T23
T03p=T03*T33p
T04=T03p*T3p4
T05=T04*T45
T05p=T05*T55p
T06=T05p*T5p6
T06p=T06*T66p
T07=T06p*T6p7


pub = rospy.Publisher('joint_states', JointState, queue_size=1)
pos = rospy.Publisher('position', Point, queue_size=1)
position = Point()
header = Header()
seq = 0
header.frame_id=''
joint_msg=JointState()
joint_msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
angles=[0.0,0.0,0.0,0.0,0.0,0.0]
def callback(data):
    angles[0]=data.linear.x
    angles[1]=data.linear.y
    angles[2]=data.linear.z
    angles[3]=data.angular.x
    angles[4]=data.angular.y
    angles[5]=data.angular.z
    global seq
    seq=seq+1
    header.seq = seq
    header.stamp = rospy.get_rostime()
    joint_msg.header=header
    joint_msg.position=angles
    pub.publish(joint_msg)
    T07n=T07.subs([(d1,data.linear.x),(t1,data.linear.y),(t2,data.linear.z),(t3,data.angular.x),(t4,data.angular.y),(t5,data.angular.z)])
    position.x= T07n[0,3]
    position.y= T07n[1,3]
    position.z= T07n[2,3]
    pos.publish(position)
    print(angles)
def listener():

   
    rospy.init_node('ej2', anonymous=True)
   
    rospy.Subscriber("angles", Twist, callback)
    print('px=', T07[0,3])
    print('py=', T07[1,3])
    print('pz=', T07[2,3])

    rospy.spin() #keep alive
if __name__ == '__main__':
    listener()
