'''
  Wrapper around willow's app manager for the concert.
'''

##############################################################################
# Imports
##############################################################################

import os
import uuid

import roslib; roslib.load_manifest('rocon_linux_app_manager')
import rospy
import willow_app_manager
import concert_comms
from concert_comms import msg
#from concert_comms.msg import * 

##############################################################################
# Classes
##############################################################################

class Invitation():
    def __init__(self, concert_master_ip, concert_master_port, concert_namespace, concert_master_uri ):
        self.concert_namespace = concert_namespace
        self.concert_master_ip = concert_master_ip
        self.concert_master_port = concert_master_port
        self.concert_master_uri = concert_master_uri

'''
  Parameters:
  
  app_list_dir : if not set, uses the default ~/.ros/app_manager/app_list
  app_store_url : if not set, uses the default ~/.ros/app_manager/app_store/app_store.yaml
'''        
class AppManagement():
    '''
      Wrap the willow app manager to provide concert app functionality.
      
      @arg robot : type robot this app manager is running on (part of the platform-system-robot triple).
      @arg suggested_name : name to suggest for this platform
    '''
    def __init__(self, robot, suggested_name):
        # Variables
        self.concert_master_ip = None
        self.concert_master_port = None
        self.concert_namespace = None # namespace to expose on the concert master (must not conflict!)
        self.app_list = None
        self.app_store = None
        self.app_manager = None
        self.invitations = []

        self.platform_info = concert_comms.msg.PlatformInfo()
        self.platform_info.platform = concert_comms.msg.PlatformInfo.PLATFORM_LINUX
        self.platform_info.system = concert_comms.msg.PlatformInfo.SYSTEM_ROS
        self.platform_info.robot = robot
        key = uuid.uuid4() # random 16 byte string, alternatively uuid.getnode() returns a hash based on the mac address, uuid.uid1() based on localhost and time
        self.platform_info.key = key.hex # convert the uuid key into a hex string
        self.platform_info.suggested_name = suggested_name
        self.platform_info.unique_name = ""
        
        # Paths
        self.app_manager_home = os.path.join(roslib.rosenv.get_ros_home(),"app_manager")
        #self.app_list_directory = app_manager.get_default_applist_directory() # /etc/robot/apps
        self.app_list_directory = rospy.get_param("~app_list_dir", os.path.join(self.app_manager_home,"app_list"))
        if not os.path.exists(self.app_manager_home):
            os.makedirs(self.app_manager_home)
        if not os.path.exists(self.app_list_directory):
            os.makedirs(self.app_list_directory)
        self.app_store_url = rospy.get_param("~app_store_url", os.path.join(self.app_manager_home,"app_store","app_store.yaml"))
        self.app_store_directory = os.path.join(self.app_manager_home,"app_store")
        self.app_store_app_list_directory = os.path.join(self.app_store_directory, "installed")
        if not os.path.exists(self.app_store_app_list_directory):
            os.makedirs(self.app_store_app_list_directory)
        
        self.app_list = willow_app_manager.AppList([])
        self.app_list.add_directory(self.app_list_directory)
        self.app_list.add_directory(self.app_store_app_list_directory)
        
        try:
            self.app_store = willow_app_manager.Exchange(self.app_store_url, self.app_store_directory)
            self.app_store.update() # Let's get an update regardless
        except willow_app_manager.AppException as e:
            rospy.logerror("App Manager: failed to load the app store [%s][%s]"%(e,rospy.get_name()))
            
    def __repr__(self):
        if self.concert_namespace is not None:
            string_representation = "  Namespace  : " + self.concert_namespace + "\n"
        else:
            string_representation = "  Namespace : unassigned\n"
        string_representation += "  Key: " + self.platform_info.key + "\n"
        string_representation += "  Paths:\n"
        string_representation += "    Home    : " + self.app_manager_home + "\n"
        string_representation += "    AppList : " + self.app_list_directory + "\n"
        string_representation += "    AppStore: " + self.app_store_directory + "\n"
        string_representation += "            : " + self.app_store_app_list_directory + "\n"
        string_representation += "  URLs:\n"
        string_representation += "    AppStore: " + self.app_store_url + "\n"
        return string_representation
    
    def receive_invitation_callback(self, concert_master_ip, concert_master_port, concert_namespace ):
        '''
          We're directly calling this from the invite() method of the backdoor xmlrpc server.
          This may not be a good thing - perhaps better to have an event polling loop and 
          pick up on the invite there.
        '''
        concert_master_uri = "http://" + concert_master_ip+":" + str(concert_master_port)
        rospy.loginfo("App Manager: stored concert invitation [%s][%s]"%(concert_namespace,concert_master_uri))
        # To just receive the invitation and process in the spin loop:.
        self.invitations.append(Invitation(concert_master_ip, concert_master_port, concert_namespace, concert_master_uri))
        return True
        # To process in the spin loop, process the invitation
        # return self.process_invitation(Invitation(concert_master_ip, concert_master_port, concert_namespace, concert_master_uri))

    def process_invitation(self, invitation):
        if not self.app_manager is None:
            rospy.loginfo("App Manager: discarding invitation, already connected to concert [%s:%s]"%(invitation.concert_master_ip,invitation.concert_master_port))
            return False
        rospy.loginfo("App Manager: connecting to concert [%s]"%invitation.concert_master_uri)
        
        #self.app_manager = app_manager.AppManager(invitation.concert_namespace, invitation.concert_master_uri, self.app_list, None)
        self.app_manager = willow_app_manager.AppManager(invitation.concert_namespace, invitation.concert_master_uri, self.app_list, self.app_store)
        rospy.on_shutdown(self.app_manager.shutdown)
        self.concert_master_ip = invitation.concert_master_ip
        self.concert_master_port = invitation.concert_master_port
        self.concert_namespace = invitation.concert_namespace
        # Our own publications on top
        rospy.sleep(1.0)
        # We latch this one, so whenever anyone connects to it, they will immediately get the platform information
        # Note that we only publish this one once PlatformInfo is fully fleshed out after connection to the concert
        # Note also that the concert uses this one to watchdog!
        self._platform_info_pub = rospy.Publisher(self.app_manager.scoped_name('platform_info'), concert_comms.msg.PlatformInfo, latch=True)
        self.app_manager._api_sync.local_manager.subscribe(self._platform_info_pub.resolved_name)

        # App store pubs and services on top
        self.app_manager._api_sync.local_manager.subscribe(self.app_manager._exchange_list_apps_pub.resolved_name)
        if self.app_store is not None:
            self.app_manager._api_sync.local_service_names.append(self.app_manager._list_exchange_apps_srv.resolved_name)
            self.app_manager._api_sync.local_service_names.append(self.app_manager._get_app_details_srv.resolved_name)
            self.app_manager._api_sync.local_service_names.append(self.app_manager._install_app_srv.resolved_name)
            self.app_manager._api_sync.local_service_names.append(self.app_manager._uninstall_app_srv.resolved_name)
        
        self. platform_info.unique_name = self.concert_namespace
        self._platform_info_pub.publish(self.platform_info)
        rospy.loginfo("App Manager: initialised: \n\n%s"%self.__repr__())
        return True
            
    def spin(self):
        '''
          Process events set in motion by the backdoor xmlrpc server.
          
          Currently not using directly - processing connections in the callback.
        '''
        while not rospy.is_shutdown():
            while len(self.invitations) > 0 :
                self.process_invitation(self.invitations.pop())
            rospy.sleep(1.0)
