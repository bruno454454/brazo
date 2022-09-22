#!/usr/bin/env python
import rospy
from sympy import*
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')
t4=Symbol('t4')


def dh_matrix (t,d,a,aph):

	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],[sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],[0,sin(aph), cos(aph), d],[0, 0, 0, 1]])
	return T



T01=dh_matrix(0,0,0,0)
T12=dh_matrix(t1+pi,0.4,0,pi/2)
T22p=dh_matrix(t2+pi,0.3,0.25,0)
T2p3=dh_matrix(pi/2,0,-0.35,pi/2)
T34=dh_matrix(t3-pi/2,0.45,0,pi/2)
T45=dh_matrix(t4,0,0.3,0)

T02=T01*T12
T02p=T02*T22p
T03=T02p*T2p3
T04=T03*T34
T05=T04*T45


pub = rospy.Publisher('joint_states', JointState, queue_size=1)
pos = rospy.Publisher('position', Point, queue_size=1)
position = Point()
header = Header()
seq = 0
header.frame_id=''
joint_msg=JointState()
joint_msg.name = ['joint1','joint2','joint3','joint4']
angles=[0.0,0.0,0.0,0.0]
def callback(data):
    angles[0]=data.linear.x
    angles[1]=data.linear.y
    angles[2]=data.linear.z
    angles[3]=data.angular.x
    global seq
    seq=seq+1
    header.seq = seq
    header.stamp = rospy.get_rostime()
    joint_msg.header=header
    joint_msg.position=angles
    pub.publish(joint_msg)
    T05n=T05.subs([(t1,data.linear.x),(t2,data.linear.y),(t3,data.linear.z),(t4,data.angular.x)])#ultimo resultado de las multiplicaciones= T02p
    position.x= T05n[0,3]
    position.y= T05n[1,3]
    position.z= T05n[2,3]
    pos.publish(position)
    print(angles)
def listener():

   
    rospy.init_node('robot', anonymous=True)
   
    rospy.Subscriber("angles", Twist, callback)
    print('px:',T05[0,3]) #tiene que ser numerico
    print('py:',T05[1,3]) #tiene que ser numerico
    print('pz:',T05[2,3]) #tiene que ser numerico



    rospy.spin() #keep alive
if __name__ == '__main__':
    listener()
