#!/usr/bin/env python
import rospy
from sympy import*
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')

l1=Symbol('L1')
l2=Symbol('L2')
l3=Symbol('L3')

def dh_matrix (t,d,a,aph):

	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],
              [sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],
              [0, sin(aph), cos(aph), d],
              [0, 0, 0, 1]])

	return T

T01=dh_matrix(0,0.1,0,0)
T12=dh_matrix(t1+pi,0.2,-0.2,pi/2)
T23=dh_matrix(t2+pi/2,0,0.4,pi)
T34=dh_matrix(t3-pi/2,0,0.4,0)

T02=T01*T12
T03=T02*T23
T04=T03*T34


pub = rospy.Publisher('joint_states', JointState, queue_size=1)
pos = rospy.Publisher('position', Point, queue_size=1)
position = Point()
header = Header()
seq = 0
header.frame_id=''
joint_msg=JointState()
joint_msg.name = ['joint1','joint2','joint3'] #modify

angles=[0.0,0.0,0.0] #modify

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

    T04n=T04.subs([(t1,data.x),
                   (t2,data.y),
                   (t3,data.z)]) #modify
                   
    position.x= T04n[0,3]   #modify T0Xn
    position.y= T04n[1,3]   #modify T0Xn
    position.z= T04n[2,3]   #modify T0Xn
    print(T04n)
    pos.publish(position)
    print(angles)

def listener():

    rospy.init_node('for_kin_auxiliar_class3', anonymous=True)   
    rospy.Subscriber("angles", Point, callback)

    rospy.spin() #keep alive
    
if __name__ == '__main__':
    listener()

