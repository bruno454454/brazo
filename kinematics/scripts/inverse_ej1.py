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
d2=Symbol('t1')
d3=Symbol('t2')

header = Header()
joint_msg = JointState()
seq = 0
header.frame_id = ''
joint_msg.name = ['joint1','joint2','joint3']
angles = [0.0, 0.0, 0.0]
alpha=0.3 #learning rate
iterations = 100
pub = rospy.Publisher('joint_states', JointState, queue_size=10)

px=d2 + 0.55

py=-d1

pz=d3 + 0.4

J=Matrix([[diff(px,d1),diff(px,t1),diff(px,t2)],[diff(py,d1),diff(py,t1),diff(py,t2)],[diff(pz,d1),diff(pz,t1),diff(pz,t2)]])

def callback(data): #callback function
	
	target=Matrix([data.x,data.y,data.z])
	ti=Matrix([random(),random(),random()])
	for i in range(0,iterations):
		cp= Matrix([px.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2])]),py.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2])]),pz.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2])])])
		e=target-cp
		Jsubs=J.subs([(d1,ti[0]),(t1,ti[1]),(t2,ti[2])])
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


		joint_msg.header = header
		joint_msg.position = angles
		pub.publish(joint_msg)
		seq += 1
		time.sleep(0.5)	
def subscriber():
    rospy.init_node('inverse_ej1', anonymous=True)
    rospy.Subscriber("position", Point, callback) 
    rospy.spin()

if __name__ == '__main__':
    subscriber()

