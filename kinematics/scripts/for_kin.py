#!/usr/bin/env python
import rospy
from sympy import*
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion #hasta 4 variables de movimiento(data.x  data.y  data.z  data.w)
from geometry_msgs.msg import Twist #hasta 6 variables de movimiento (data.linear.x data.linear.y data.linear.z data.angular.x data.angular.y data.angular.z)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

t1=Symbol('t1')
t2=Symbol('t2')


def dh_matrix (t,d,a,aph):

	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],[sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],[0,sin(aph), cos(aph), d],[0, 0, 0, 1]])
	return T



T01=dh_matrix(0,0,0,0)
T12=dh_matrix(t1-pi,0.4,0,pi/2)
T23=dh_matrix(t2,0,-0.4,0)

T02=T01*T12
T03=T02*T23


pub = rospy.Publisher('joint_states', JointState, queue_size=1)
pos = rospy.Publisher('position', Point, queue_size=1)
position = Point()
header = Header()
seq = 0
header.frame_id=''
joint_msg=JointState()
joint_msg.name = ['joint1','joint2']
angles=[0.0,0.0]
def callback(data):
    angles[0]=data.x
    angles[1]=data.y
    global seq
    seq=seq+1
    header.seq = seq
    header.stamp = rospy.get_rostime()
    joint_msg.header=header
    joint_msg.position=angles
    pub.publish(joint_msg)
    T03n=T03.subs([(t1,data.x),(t2,data.y)])
    position.x= T03n[0,3]
    position.y= T03n[1,3]
    position.z= T03n[2,3]
    pos.publish(position)
    print(angles)
def listener():

   
    rospy.init_node('for_kin', anonymous=True)
   
    rospy.Subscriber("angles", Point, callback)


    rospy.spin() #keep alive
if __name__ == '__main__':
    listener()
