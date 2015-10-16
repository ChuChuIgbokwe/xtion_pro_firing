#!/usr/bin/env python
import rospy
from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point
# from cv_bridge import CVBridge, CVBridgeError
from skeletonmsgs_nu.msg import Skeletons,Skeleton

pub = rospy.Publisher('rocket_command',String, queue_size=1)
fire_bool = False

def control_rocket(command):
        p = String()
        p.data = command    
        pub.publish(p)

def callback(data):
        global fire_bool
        #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        # rospy.log('callback triggered')
        right_hand = data.skeletons[0].right_hand.transform.translation.y
	left_hand = data.skeletons[0].left_hand.transform.translation.y
        right_hip = data.skeletons[0].right_hip.transform.translation.y
	left_hip = data.skeletons[0].left_hip.transform.translation.y
	right_shoulder_pan = data.skeletons[0].right_shoulder.transform.translation.y
	left_shoulder_pan = data.skeletons[0].left_shoulder.transform.translation.x
	left_shoulder_tilt = data.skeletons[0].left_shoulder.transform.translation.y
	right_shoulder_tilt = data.skeletons[0].right_shoulder.transform.translation.y
	left_hand_pan = data.skeletons[0].left_hand.transform.translation.x
	left_hand_tilt = data.skeletons[0].left_hand.transform.translation.y
	right_hand_tilt = data.skeletons[0].right_hand.transform.translation.y
	right_hand_hadouken = data.skeletons[0].right_hand.transform.translation.z
	left_hand_hadouken = data.skeletons[0].left_hand.transform.translation.z
	right_hip_hadouken = data.skeletons[0].right_hip.transform.translation.z
        '''if right_hand > right_hip:
                fire_bool = True
        elif right_hand < right_hip and fire_bool:
                control_rocket('fire')
                rospy.loginfo("FIRING!")
                fire_bool = False
                #rospy.sleep(3.0)
                rospy.loginfo("DONE")'''
	#elif right_hand and left_hand > right_shoulder_pan and left_shoulder_pan:
	#	control_rocket('stop')
	
	if left_shoulder_pan > left_hand_pan and right_shoulder_tilt < right_hand_tilt:
		control_rocket('left')
	elif left_shoulder_pan < left_hand_pan and right_shoulder_tilt < right_hand_tilt:
		control_rocket('right')

	if left_shoulder_tilt > left_hand_tilt and right_shoulder_tilt > right_hand_tilt:
		control_rocket('up')
	elif left_shoulder_tilt < left_hand_tilt and right_shoulder_tilt > right_hand_tilt:
		control_rocket('down')
	elif left_hand < left_hip and right_hand < right_hip:
		control_rocket('stop')
	if right_hand_hadouken > right_hip_hadouken and left_hand_hadouken > right_hip_hadouken:
		control_rocket('fire')
		rospy.sleep(2.0)
		


	'''if right_shoulder_tilt > right_hand_tiltr:
		control_rocket('up')
	elif right_shoulder_tilt < right_hand_tilt:
		control_rocket('down')'''


	

	
        
   
def listener():

        # in ROS, nodes are unique named. If two nodes with the same
        # node are launched, the previous one is kicked off. The 
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaenously.
        rospy.init_node('image_data', anonymous=True)

        rospy.Subscriber("skeletons", Skeletons, callback, queue_size=1)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
if __name__ == '__main__':
        listener()
