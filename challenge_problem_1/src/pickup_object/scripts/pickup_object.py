#!/usr/bin/env python

import rospy

from add_ints.srv import AddTwoInts
from move_gripper.srv import CloseGripper
from move_base.srv import MoveBase, RotateBase
from move_arm.srv import MoveRightArm, MoveLeftArm
#from move_head.srv import MoveHead

add_int = None
mover = None
move_gripper = None
rotater = None
move_left_arm = None
move_right_arm = None
#mover_head = None

def init_gripper_service():
	rospy.wait_for_service('move_gripper')
	return rospy.ServiceProxy('move_gripper', CloseGripper)

def init_move_base():
	rospy.wait_for_service('move_base')
	mover = rospy.ServiceProxy('move_base', MoveBase)
	return mover

def init_rotate_base():
	rospy.wait_for_service('rotate_base')
	rotater = rospy.ServiceProxy('rotate_base', RotateBase)
	return rotater

def init_add_ints():
	rospy.wait_for_service('add_two_ints')
	add_int = rospy.ServiceProxy('add_two_ints', AddTwoInts)
	return add_int

def init_left_arm_service():
	rospy.wait_for_service('move_left_arm')
	return rospy.ServiceProxy('move_left_arm', MoveLeftArm)

def init_right_arm_service():
	rospy.wait_for_service('move_right_arm')
	return rospy.ServiceProxy('move_right_arm', MoveRightArm)

'''
def init_move_head_service():
	rospy.wait_for_service('move_head')
	rotater = rospy.ServiceProxy('move_head', MoveHade)
	return mover_head
'''

def move_gripper(should_close):
	try:
		mb = move_gripper(should_close)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def close_gripper():
	move_gripper(True)

def open_gripper():
	move_gripper(False)

def add_two_ints(a, b):
	try:
		sum_two = add_int(a,b)
		print sum_two
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def move_base(d):
	try:
		mv = mover(d)
		print "What smooth movements"
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def rotate_base(angle):
	try:
		rb = rotater(angle)
		print "What fluid rotation"
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def move_arm(right_arm, x, y, z, ow, ox, oy, oz):
	try:	
		if right_arm:
			ma = move_right_arm(x,y,z,ow,ox,oy,oz)
		else:
			ma = move_left_arm(x,y,z,ow,ox,oy,oz)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))

def move_left_arm(x, y, z, ow, ox, oy, oz):
	move_arm(False, x, y, z, ow, ox, oy, oz)

def move_right_arm(x, y, z, ow, ox, oy, oz):
	move_arm(True, x, y, z, ow, ox, oy, oz)

def hover_left_arm(x, y, z):
	move_arm(False, x, y, z, 1, 1, 1, 0)

def hover_right_arm(x, y, z):
	move_arm(True, x, y, z + 1, 1, 0, 0, -1)
			
def init_services():
	'''	
	global add_int
	add_int = init_add_ints()
	'''	
	global mover
	mover = init_move_base()
	global move_left_arm
	move_left_arm = init_left_arm_service()
	global move_right_arm
	move_right_arm = init_right_arm_service()
	global move_gripper
	move_gripper = init_gripper_service()
	global rotater
	rotater = init_rotate_base()
	#global mover_head
	#mover_head = init_move_head_service()
    
if __name__ == '__main__':
    	rospy.init_node('Hair_of_Renato_client', anonymous=True)

	init_services()

	#int_1 = 7;
	#int_2 = 11;	
	#add_two_ints(int_1, int_2)

	#move_base(.1)
	basex = 0
	basey = 0

	print "Tyring to move the arm"
	x = .38
	y = .73
	z = .6
	ow = 1
	ox = 1
	oy = 1
	oz = 1
        #open_gripper()
        #rospy.sleep(1.2)
	hover_left_arm(0,0,0)
	'''	
	rospy.sleep(5)
	
	print "Did it move"
	
	basex += 1.158
	move_base(basex)
	print "Hopefully the gripper has opened"
	close_gripper()
	print "Hopefully the gripper has closed"
	rospy.sleep(15)
	z = .72
	y = .65
	# WHY CAN I ONLY MOVE THE ARM ONCE?????????
	#print "Move your arm you stupid robot"	
	move_left_arm(basex+x,y,z,ow,ox,oy,oz)
	print "The hair of Renato"

	#rospy.loginfo("Let's see if this works")
	'''
