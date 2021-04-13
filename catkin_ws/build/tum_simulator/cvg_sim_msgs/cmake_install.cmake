# Install script for directory: /root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/share/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/msg" TYPE FILE FILES
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/Altimeter.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/Altitude.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/AttitudeCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/Compass.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/ControllerState.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/HeadingCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/HeightCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/MotorCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/MotorPWM.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/MotorStatus.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/PositionXYCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/RawImu.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/RawMagnetic.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/RawRC.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/RC.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/RuddersCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/ServoCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/Supply.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/ThrustCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/VelocityXYCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/VelocityZCommand.msg"
    "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/msg/YawrateCommand.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/cmake" TYPE FILE FILES "/root/share/catkin_ws/build/tum_simulator/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/root/share/catkin_ws/devel/include/cvg_sim_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/root/share/catkin_ws/devel/share/roseus/ros/cvg_sim_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/root/share/catkin_ws/devel/share/common-lisp/ros/cvg_sim_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/root/share/catkin_ws/devel/share/gennodejs/ros/cvg_sim_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/root/share/catkin_ws/devel/lib/python2.7/dist-packages/cvg_sim_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/root/share/catkin_ws/devel/lib/python2.7/dist-packages/cvg_sim_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/root/share/catkin_ws/build/tum_simulator/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/cmake" TYPE FILE FILES "/root/share/catkin_ws/build/tum_simulator/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs/cmake" TYPE FILE FILES
    "/root/share/catkin_ws/build/tum_simulator/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgsConfig.cmake"
    "/root/share/catkin_ws/build/tum_simulator/cvg_sim_msgs/catkin_generated/installspace/cvg_sim_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvg_sim_msgs" TYPE FILE FILES "/root/share/catkin_ws/src/tum_simulator/cvg_sim_msgs/package.xml")
endif()

