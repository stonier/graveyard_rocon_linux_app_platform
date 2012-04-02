#!/usr/bin/env python

import roslib; roslib.load_manifest('rocon_linux_app_manager')
import rospy

from ros import rocon_linux_app_manager

if __name__ == '__main__':
    rocon_linux_app_manager.main()
