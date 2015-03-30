cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning_interface
    roscpp
    pluginlib
    openrave_catkin
)
catkin_package()

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenRAVE REQUIRED)

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

include_directories(
    "include/"
    ${Boost_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
    ${YamlCpp_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

# OpenRAVE plugin.
openrave_plugin("${PROJECT_NAME}_plugin"
    src/BenchmarksModule.cpp
    src/CollisionCheckingBenchmark.cpp
    src/DataUtils.cpp
    src/KinematicBenchmarks.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    profiler
    ${OpenRAVE_LIBRARIES}
    ${YamlCpp_LIBRARIES}
)


# Profiling benchmark.
add_executable(run_profiler
    src/run_benchmark.cpp
    src/DataUtils.cpp
)
target_link_libraries(run_profiler
    profiler
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${YamlCpp_LIBRARIES}
    openrave0.9-core
    ${OpenRAVE_LIBRARIES}
)

# MoveIt plugin.
add_executable(run_moveit_benchmark
    src/MoveItModule.cpp
)
target_link_libraries(run_moveit_benchmark
    ${catkin_LIBRARIES}
    ${YamlCpp_LIBRARIES}
)

# Memory Leak
add_executable(run_memory_leak
    src/memory_leak.cpp
)
target_link_libraries(run_memory_leak
    openrave${OpenRAVE_LIBRARY_SUFFIX}-core
    ${Boost_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
)

# Installation.
install(TARGETS "${PROJECT_NAME}_plugin"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
install(TARGETS run_moveit_benchmark
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS "scripts/run_benchmark.py"
    DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
