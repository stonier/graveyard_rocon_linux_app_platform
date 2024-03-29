##############################################################################
# Rosbuild2
##############################################################################

if(ROSBUILD)
  include(rosbuild.cmake OPTIONAL)
  return()
endif()

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)

##############################################################################
# Ros Initialisation
##############################################################################

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

##############################################################################
# Qt Environment
##############################################################################

rosbuild_include(qt_build qt-ros)
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here (QtOpengl)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/rocon_projector/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
file(GLOB QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# RPaths
##############################################################################

# Don't strip rpaths
set(CMAKE_INSTALLL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

##############################################################################
# Binaries
##############################################################################

rosbuild_add_executable(rocon_projector ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(rocon_projector ${QT_LIBRARIES})

rosbuild_add_executable(test_background_change src/tests/test_background_change.cpp)

install(TARGETS rocon_projector DESTINATION ${PROJECT_NAME}/bin)

###############################################################################
# Version
###############################################################################

set(PROJECT_VERSION "0.10.4")
set(PROJECT_VERSION_MAJOR "0")
set(PROJECT_VERSION_MINOR "10")
set(PROJECT_VERSION_PATCH "4")

###############################################################################
# Rocon
###############################################################################

rosbuild_include(rocon_build rocon_packaging)

###############################################################################
# Project
###############################################################################

add_subdirectory(src)
add_subdirectory(resources)
rocon_apps_roll_deb_pkg()
rocon_upload_app_data_to_yujin_repo(rocon_app_projector projector)

install(FILES CMakeLists.txt Makefile manifest.xml readme.txt DESTINATION ${PROJECT_NAME} )
