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
t1 = Symbol('t1')
t2 = Symbol('t2')
t3 = Symbol('t3')
t4 = Symbol('t4')
t5 = Symbol('t5')
header = Header()
joint_msg = JointState()
seq = 0
header.frame_id = ''
joint_msg.name = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
alpha=0.3 #learning rate
iterations = 100
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
px=0.09465*(sin(t2)*sin(t3)*cos(t1) - cos(t1)*cos(t2)*cos(t3))*sin(t4) - 0.09465*(sin(t2)*cos(t1)*cos(t3) + sin(t3)*cos(t1)*cos(t2))*cos(t4) - 0.10915*sin(t1) - 0.39225*sin(t2)*sin(t3)*cos(t1) + 0.39225*cos(t1)*cos(t2)*cos(t3) + 0.425*cos(t1)*cos(t2)
py=0.09465*(sin(t1)*sin(t2)*sin(t3) - sin(t1)*cos(t2)*cos(t3))*sin(t4) - 0.09465*(sin(t1)*sin(t2)*cos(t3) + sin(t1)*sin(t3)*cos(t2))*cos(t4) - 0.39225*sin(t1)*sin(t2)*sin(t3) + 0.39225*sin(t1)*cos(t2)*cos(t3) + 0.425*sin(t1)*cos(t2) + 0.10915*cos(t1)
pz=-0.09465*(-sin(t2)*sin(t3) + cos(t2)*cos(t3))*cos(t4) + 0.09465*(sin(t2)*cos(t3) + sin(t3)*cos(t2))*sin(t4) - 0.39225*sin(t2)*cos(t3) - 0.425*sin(t2) - 0.39225*sin(t3)*cos(t2) + 0.089159
J=Matrix([[diff(px,t1),diff(px,t2),diff(px,t3),diff(px,t4)],[diff(py,t1),diff(py,t2),diff(py,t3),diff(py,t4)],[diff(pz,t1),diff(pz,t2),diff(pz,t3),diff(pz,t4)]])
def callback(data): #callback function
	
	target=Matrix([data.x,data.y,data.z])
	ti=Matrix([random(),random(),random(),random()])
	for i in range(0,iterations):
		cp= Matrix([px.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])]),py.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])]),pz.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])])])
		e=target-Matrix([px.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])]),py.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])]),pz.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])])])	
		Jsubs=J.subs([(t1,ti[0]),(t2,ti[1]),(t3,ti[2]),(t4,ti[3])])
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
		joint_msg.header = header
		joint_msg.position = angles
		pub.publish(joint_msg)
		seq += 1
		time.sleep(0.5)	
def subscriber():
    rospy.init_node('inverse_kin', anonymous=True)
    rospy.Subscriber("position", Point, callback) 
    rospy.spin()

if __name__ == '__main__':
    subscriber()

