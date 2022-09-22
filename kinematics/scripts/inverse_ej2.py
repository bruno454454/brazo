#!/usr/bin/env python
import rospy
import time
from sympy import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point 
from std_msgs.msg import String
from std_msgs.msg import Int32
from random import random
d1=Symbol('d1')
t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')
t4=Symbol('t4')
t5=Symbol('t5')

header = Header()
joint_msg = JointState()
seq = 0
header.frame_id = ''
joint_msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
angles = [0.0, 0.0, 0.0,0.0,0.0,0.0]
alpha=0.3 #learning rate
iterations = 100
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

px= d1 - 0.2*(-(sin(t1)*sin(t3) + sin(t2)*cos(t1)*cos(t3))*sin(t4) - cos(t1)*cos(t2)*cos(t4))*cos(t5) + 0.2*(-(sin(t1)*sin(t3) + sin(t2)*cos(t1)*cos(t3))*cos(t4) + sin(t4)*cos(t1)*cos(t2))*sin(t5) + 0.1*(sin(t1)*sin(t3) + sin(t2)*cos(t1)*cos(t3))*sin(t4) - 0.2*sin(t2)*cos(t1) + 0.1*cos(t1)*cos(t2)*cos(t4) + 0.6*cos(t1)*cos(t2)

py= -0.2*(-(sin(t1)*sin(t2)*cos(t3) - sin(t3)*cos(t1))*sin(t4) - sin(t1)*cos(t2)*cos(t4))*cos(t5) + 0.2*(-(sin(t1)*sin(t2)*cos(t3) - sin(t3)*cos(t1))*cos(t4) + sin(t1)*sin(t4)*cos(t2))*sin(t5) + 0.1*(sin(t1)*sin(t2)*cos(t3) - sin(t3)*cos(t1))*sin(t4) - 0.2*sin(t1)*sin(t2) + 0.1*sin(t1)*cos(t2)*cos(t4) + 0.6*sin(t1)*cos(t2)


pz=0.2*(sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4))*sin(t5) - 0.2*(-sin(t2)*cos(t4) + sin(t4)*cos(t2)*cos(t3))*cos(t5) + 0.1*sin(t2)*cos(t4) + 0.6*sin(t2) - 0.1*sin(t4)*cos(t2)*cos(t3) + 0.2*cos(t2) + 0.65

J=Matrix([[diff(px,d1),diff(px,t1),diff(px,t2),diff(px,t3),diff(px,t4),diff(px,t5)],[diff(py,d1),diff(py,t1),diff(py,t2),diff(py,t3),diff(py,t4),diff(py,t5)],[diff(pz,d1),diff(pz,t1),diff(pz,t2),diff(pz,t3),diff(pz,t4),diff(pz,t5)]])

def callback(data): #callback function
	
	target=Matrix([data.x,data.y,data.z])
	ti=Matrix([random(),random(),random(),random(),random(),random()])
	for i in range(0,iterations):
		cp= Matrix([px.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2]),(t3,ti[3]),(t4,ti[4]),(t5,ti[5])]),py.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2]),(t3,ti[3]),(t4,ti[4]),(t5,ti[5])]),pz.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2]),(t3,ti[3]),(t4,ti[4]),(t5,ti[5])])])
		e=target-cp
		Jsubs=J.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2]),(t3,ti[3]),(t4,ti[4]),(t5,ti[5])])
		Jinv=Jsubs.H*(Jsubs*Jsubs.H)**-1
		dt=Jinv*e
		ti=ti+alpha*dt
		#print (cp)
		global seq
		header.seq = seq
		header.stamp = rospy.Time.now()
		angles[0] = ti[0]
		angles[1] = ti[1]
		angles[2] = ti[2]
		angles[3] = ti[3]
		angles[4] = ti[4]
		angles[5] = ti[5]


		joint_msg.header = header
		joint_msg.position = angles
		pub.publish(joint_msg)
		seq += 1
		time.sleep(0.5)	
def subscriber():
    rospy.init_node('inverse_ej2', anonymous=True)
    rospy.Subscriber("position", Point, callback) 
    rospy.spin()

if __name__ == '__main__':
    subscriber()

