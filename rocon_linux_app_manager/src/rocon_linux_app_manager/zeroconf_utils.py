'''
Created on 10/08/2011

@author: snorri
'''
##############################################################################
# Imports
##############################################################################

import roslib; roslib.load_manifest('rocon_linux_app_manager')
import rospy

from zeroconf_msgs.msg import *
from zeroconf_msgs.srv import *

##############################################################################
# Functions
##############################################################################

def ipv4_protocol():
    return zeroconf_msgs.msg.Protocols.IPV4

def listen_for_concerts():
    '''
        Tell the app manager's zeroconf node to listen for app managers.
    '''
    try:
        rospy.wait_for_service('add_listener', 30.0)
        add_listener=rospy.ServiceProxy('add_listener', AddListener)
        request = zeroconf_msgs.srv.AddListenerRequest()
        request.service_type = '_concert-master._tcp'
        response = add_listener(request)
        if response.result == False:
            rospy.logerr("Conductor: couldn't add a listener to the concert master's zeroconf node.")
            return False
    except rospy.ROSException:
        rospy.logerr("Conductor: timed out waiting for the concert master to advertise itself.")
        return False
    return True

def discover_concerts():
    '''
    This is a one-shot call to discover concerts.
    '''
    rospy.wait_for_service('list_discovered_services')
        
    try:
        discover_concerts = rospy.ServiceProxy('list_discovered_services', zeroconf_msgs.srv.ListDiscoveredServices)
        request = zeroconf_msgs.srv.ListDiscoveredServicesRequest()
        request.service_type = "_concert-master._tcp"
        response = discover_concerts(request)
        rospy.logdebug("AppManager : found concerts")
        return response.services
    except rospy.ServiceException, error:
        rospy.logwarn("AppManager : could not get the list of concerts via zeroconf [%s]"%error)

def advertise_app_manager(name, domain, port):
    '''
    First get the parameter for this app manager's name, then advertise.
    '''
    try:
        rospy.wait_for_service('add_service', 20.0)
    except rospy.ROSException:
        rospy.logerr("App Manager: timeout while trying to advertise the app manager on zeroconf")
        return False
    
    request = zeroconf_msgs.srv.AddServiceRequest()
    request.service.name = name
    request.service.domain = domain
    request.service.type = "_app-manager._tcp"
    request.service.port = port
    
    try:
        advertise_app_manager = rospy.ServiceProxy('add_service', zeroconf_msgs.srv.AddService)
        response = advertise_app_manager(request)
        if response.result:
            rospy.loginfo("App Manager: advertising service [%s][%s][%s]"%(request.service.name, request.service.domain, request.service.port))
        else:
            rospy.logwarn("App Manager : failed to advertise this app manager on zeroconf")
    except rospy.ServiceException, error:
        rospy.logwarn("App Manager : exception thrown when advertising this app manager on zeroconf [%s]"%error)
