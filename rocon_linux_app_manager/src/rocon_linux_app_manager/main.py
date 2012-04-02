'''
Created on 08/08/2011

@author: snorri
'''

##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_linux_app_manager')
import rospy

# Local imports
import zeroconf_utils
from .backdoor import Backdoor
from .management import AppManagement

##############################################################################
# Main
##############################################################################

def main():
    rospy.init_node('app_manager', log_level=rospy.DEBUG)

    #####################
    # Shared Parameters Parameters
    #####################
    suggested_name = rospy.get_param("~name","ros_robot")
    robot_type = rospy.get_param("~robot","johnny5") # part of the platform-system-robot triple.
    domain = rospy.get_param("~domain","local") # for zeroconf
    unused_connection_mode = rospy.get_param("~connection_mode", "invitation")
    whitelist = rospy.get_param("~whitelist", ['Concert Master'])
    rospy.loginfo("App Manager: master whitelist [%s]"%(', '.join(whitelist)))
    blacklist = rospy.get_param("~blacklist", [])
    rospy.loginfo("App Manager: master blacklist [%s]"%(', '.join(blacklist)))
    
    management = AppManagement(robot_type, suggested_name)
    
    #####################
    # XmlRpc Server
    #####################
    backdoor = Backdoor(platform_info = management.platform_info,
                        whitelist=whitelist,
                        blacklist=blacklist,
                        receive_invitation_callback=management.receive_invitation_callback
                        )
    backdoor.start() 
    zeroconf_utils.advertise_app_manager(suggested_name, domain, backdoor.port)
    management.spin()
    backdoor.stop()

    
