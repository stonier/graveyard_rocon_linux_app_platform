# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: topics.py 11753 2010-10-25 06:23:19Z kwc $

# author: pratkanis

"""
Implements exchange part of app_manager, which handles listing of
avialable and removable applications.
"""

import subprocess
import os
import sys
import yaml
from .msg import ExchangeApp, Icon

class Exchange():
    def __init__(self, url, directory, on_error = lambda(x): None):
        self._url = url
        self._directory = directory
        self._on_error = on_error
        self._installed_apps = []
        self._available_apps = []
        self._debs = {}

        self._exchange_local = os.path.join(self._directory, "exchange.yaml")
        d = os.path.join(self._directory, "installed")
        if (not os.path.exists(d)):
            os.mkdir(d)
        self._exchange_file = os.path.join(d, "exchange.installed")

    def get_installed_version(self, deb):
        val = (subprocess.Popen(["dpkg", "-l", deb], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()
        for i in val.split('\n'):
            if (i.find(deb) > 0 and i.find("ii") == 0):
                return [s for s in i.strip().split(" ") if s][2]
        return "get_installed_version FAILED"

    def get_available_version(self, deb):
        data = subprocess.Popen(["apt-cache", "showpkg", deb], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        val = (data[0] or '').strip()
        nearing = False
        for i in val.split('\n'):
            if (nearing):
                return i.strip().split(" ")[0].strip()
            if (i.strip() == "Versions:"):
                nearing = True
        self._on_error("Failed to get available version: " + str(data))
        return "version n/a [.deb package not in repo]"

    def is_installed(self, deb):
        data = subprocess.Popen(["dpkg", "-l", deb], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        val = (data[0] or '').strip()
        for i in val.split("\n"):
            if (i.find(deb) > 0 and i.find("ii") == 0):
                return True
        self._on_error("Error getting installed packages: " + str(data))
        return False

    def get_installed_apps(self):
        return self._installed_apps
    
    def get_available_apps(self):
        return self._available_apps

    def get_app_details(self, name):
        #print "[INFO] App Store : retrieving app details from %s"%app_url
        local_path = os.path.join(self._directory, name)
        if (not os.path.exists(local_path)):
            os.makedirs(local_path)
        data = subprocess.Popen(["wget", "-O", os.path.join(local_path, "app.yaml"), (self._url.strip('/') + "/" + name + ".yaml")], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
        val = (data[0] or '').strip()        
        try:
            data = yaml.load(open(os.path.join(local_path, "app.yaml")))
            icon_url = data["icon_url"]
            icon_format = data["icon_format"]
            val = (subprocess.Popen(["wget", "-O", os.path.join(local_path, "icon" + icon_format), icon_url], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()     
            print val
        except:
            print "No icon"
        self.update_local()
        for i in self._available_apps:
            if (i.name == name):
                return i
        for i in self._installed_apps:
            if (i.name == name):
                return i
        self._on_error("Problem getting app details: " + str(data))
        return None

    def install_app(self, app):
        deb = False
        for i in self._available_apps:
            if (i.name == app):
                if (deb):
                    return False #Somehow a dupe
                deb = self._debs[i.name]
        for i in self._installed_apps:
            if (i.name == app):
                if (deb):
                    return False #Somehow a dupe                                                                                                                                                                                              
                deb = self._debs[i.name]
        if (deb == False):
            print "[INFO] App Store : can't find to install [%s]"%(app)
            return False
        print "[INFO] App Store : installing %s [%s]"%(app,deb)
        val = (subprocess.Popen(["sudo", "apt-get", "install", deb, "-y", "--allow-unauthenticated"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()
        print "===================================================================================="
        print val
        print "===================================================================================="
        self.update_local()
        for i in self._installed_apps:
            if (i.name == app):
                return True
        return False
    
    def uninstall_app(self, app):
        deb = False
        for i in self._installed_apps:
            if (i.name == app):
                if (deb):
                    return False #Somehow a dupe 
                deb = self._debs[i.name]
        if (deb == False):
            return False
        print "uninstall app"
        val = (subprocess.Popen(["sudo", "apt-get", "remove", deb, "-y"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()
        print "===================================================================================="
        print val
        print "===================================================================================="
        self.update_local()
        for i in self._available_apps:
            if (i.name == app):
                return True
        return False
    
    def update(self):
        #Call server
        print "[INFO] App Store : calling the app store server for details [%s][%s]"%(self._exchange_local,self._url)
        val = (subprocess.Popen(["wget", "-O", self._exchange_local, self._url + "/applications.yaml"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()
        if (val != "" or not os.path.exists(self._exchange_local)):
            print sys.stderr >> val
            print sys.stderr >> "Wget failed"
            return
               
        #print "[INFO] App Store : updating the local apt-get database."
        #val = (subprocess.Popen(["sudo", "apt-get", "update"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[0] or '').strip()
        self.update_local()


    def update_local(self):
        installed_apps = []
        file_apps = []
        available_apps = []
        if (not os.path.exists(self._exchange_local)):
            print "[WARN] App Store : remote app store has not been confirmed and cached."
            return
        exchange_data = yaml.load(open(self._exchange_local))
        #print exchange_data
        for app in exchange_data['apps']:
            appc = ExchangeApp()
            appc.name = app['app']
            appc.display_name = app['display']
            deb = app['debian']
            self._debs[app['app']] = deb
            appc.latest_version = self.get_available_version(deb)
	    if appc.latest_version.count("n/a") == 1:
                print "[INFO] App Store : %s listed in app store but n/a in repo."%appc.name
                continue
            appc.hidden = False
            print "[INFO] App Store : %s"%appc.name
            print "[INFO] App Store :   %s"%appc.display_name
            print "[INFO] App Store :   %s"%deb
            print "[INFO] App Store :   %s"%appc.latest_version
            try:
                if(app['hidden']):
                    appc.hidden = True
            except:
                pass

            local_path = os.path.join(self._directory, app['app'])
            if (os.path.exists(local_path)):
                format = ""
                if (os.path.exists(os.path.join(local_path, "app.yaml"))):
                    data = yaml.load(open(os.path.join(local_path, "app.yaml")))
                    try:
                        appc.description = data['description']
                    except:
                        if (appc.hidden):
                            appc.description = "Descriptionless hidden app"
                        else:
                            appc.description = "No description set, likely an error in the yaml file"
                    try:
                        format = data['icon_format']
                        print "[INFO] App Store :   %s"%appc.description
                        print "[INFO] App Store :   %s"%format
                    except:
                        pass
                if (os.path.exists(os.path.join(local_path, "icon" + format)) and format != ""):
                    icon = Icon()
                    icon.format = format.strip(".")
                    if (icon.format == "jpg"): icon.format = "jpeg"
                    icon.data = open(os.path.join(local_path, "icon" + format), "rb").read()
                    appc.icon = icon
            if (self.is_installed(deb)):
                appc.version = self.get_installed_version(deb)
                installed_apps.append(appc)
                file_apps.append(app) #Should remove debian tag?
                print "[INFO] App Store : appending %s to installed apps"%appc.name
            else:
                print "[INFO] App Store : appending %s to available apps"%appc.name
                available_apps.append(appc)
        
        f = open(self._exchange_file, "w")
        yaml.dump({"apps": file_apps}, f)
        f.close()
        self._installed_apps = installed_apps
        self._available_apps = available_apps
        
    
