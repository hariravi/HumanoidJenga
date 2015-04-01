#!/usr/bin/env python

import rospy


if __name__ == '__main__':

    #initialize the node
    rospy.init_node('pickup_object', anonymous=True)
    # Move base sample call
    rospy.wait_for_service('move_base')
    move_base = rospy.ServiceProxy('move_base', '/../../move_base/srv/MoveBase.srv' )
    distance = .75
    try:
        mb = move_base(distance)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    
    # Rotate base sample call
    rospy.wait_for_service('rotate_base')
    move_base = rospy.ServiceProxy('rotate_base', '/../../move_base/srv/RotateBase.srv' )
    # Angle in radians
    angle = 3.14
    try:
        rb = rotate_base(distance)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    rospy.loginfo("Let's see if this works")
