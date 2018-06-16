cmake_minimum_required(VERSION 2.8.3)
project(vfh_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
            geometry_msgs)

include_directories(
    include
    ${catkin_INCLUDE_DIRS})


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/VFHPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES vfh_local_planner
    CATKIN_DEPENDS
        dynamic_reconfigure
        nav_msgs
        pluginlib
        roscpp
)
add_library(vfh_local_planner src/vfh_planner_ros.cpp src/vfh_algorithm.cc)
add_dependencies(vfh_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vfh_local_planner ${catkin_LIBRARIES})

install(TARGETS vfh_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

