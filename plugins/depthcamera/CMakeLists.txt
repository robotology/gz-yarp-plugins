add_library(gz-sim-yarp-depthcamera-system SHARED DepthCamera.cc DepthCameraDriver.cpp)

target_include_directories(gz-sim-yarp-depthcamera-system PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-depthcamera-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-depthcamera-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry
    gz-sim-yarp-commons)

# Add install target
install(TARGETS gz-sim-yarp-depthcamera-system)
