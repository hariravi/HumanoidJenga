#!/usr/bin/env python

import rospy

from add_ints.srv import AddTwoInts

#AddTwoInts = '/../../add_ints/srv/AddTwoInts.srv'

if __name__ == '__main__':
	print 'checkpoint1'


    	#initialize the node
    	rospy.init_node('add_client', anonymous=True)
	print 'checkpointEOJNGKENJG'
	# Move base sample call
	rospy.wait_for_service('add_two_ints')
	print 'checkpoint3'

	add_int = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#    move_base = rospy.ServiceProxy('move_base', '/../../move_base/srv/MoveBase.srv' )
	distance = .75
	try:
		mb = add_int(4,5)
		print mb
	#mb = move_base(distance)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
    
'''
    # Rotate base sample call
    rospy.wait_for_service('rotate_base')
    move_base = rospy.ServiceProxy('rotate_base', '/../../move_base/srv/RotateBase.srv' )
    # Angle in radians
    angle = 3.14
    try:
        rb = rotate_base(distance)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
'''
	#rospy.loginfo("Let's see if this works")
