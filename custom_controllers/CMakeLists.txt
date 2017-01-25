# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(custom_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    custom_ronex_msgs
    std_srvs
    sr_ronex_msgs
    sr_ronex_hardware_interface
    sr_ronex_utilities
    sr_ronex_drivers
    pluginlib
    controller_interface
    ros_ethercat_model
    custom_ronex_interface
)

include_directories( include ${catkin_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
add_library(custom_controllers
    src/custom_controller.cpp
)
add_dependencies(custom_controllers ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_controllers ${catkin_LIBRARIES})

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        custom_ronex_msgs
        std_srvs
        sr_ronex_msgs
        sr_ronex_hardware_interface
        sr_ronex_utilities
        sr_ronex_drivers
        pluginlib
        controller_interface
        ros_ethercat_model
        custom_ronex_interface
    INCLUDE_DIRS include
    LIBRARIES sr_ronex_controllers
)

install(TARGETS custom_controllers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
