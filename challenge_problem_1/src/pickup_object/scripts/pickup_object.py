#!/usr/bin/env python

import rospy


if __name__ == '__main__':

    #initialize the node
    rospy.init_node('pickup_object', anonymous=True)
    move_base = rospy.ServiceProxy('move_base', '/../../move_base/srv/MoveBase.srv' )
    try:
        mb = move_base(.75)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
    
    rospy.loginfo("Let's see if this works")
