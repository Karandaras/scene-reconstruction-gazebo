include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources Sensors.cc
             Sensor.cc
             SensorFactory.cc
             SensorManager.cc
             CameraSensor.cc
             ContactSensor.cc
             DepthCameraSensor.cc
             ImuSensor.cc
             RaySensor.cc
             RFIDSensor.cc
             RFIDTag.cc
) 

set (headers Sensors.hh
             Sensor.hh
             SensorTypes.hh
             SensorFactory.hh
             SensorManager.hh
             CameraSensor.hh
             ContactSensor.hh
             DepthCameraSensor.hh
             ImuSensor.hh
             RaySensor.hh
             RFIDSensor.hh
             RFIDTag.hh
)

set (sensors_headers "" CACHE INTERNAL "sensors headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(sensors_headers "sensors headers"
    "#include \"sensors/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sensors.hh.in ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)


add_library(gazebo_sensors SHARED ${sources})

target_link_libraries(gazebo_sensors gazebo_common 
                                     gazebo_rendering
                                     gazebo_physics
                                     ${libtool_library} 
                                     ${Boost_LIBRARIES}
                                     ${ogre_ldflags}
                                     )

add_executable(gzsensor server.cc ../gazebo.cc)
target_link_libraries( gzsensor gazebo_common
                                gazebo_transport
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                pthread)

gz_install_executable( gzsensor )
gz_install_library(gazebo_sensors)
gz_install_includes("sensors" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)
