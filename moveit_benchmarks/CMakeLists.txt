cmake_minimum_required(VERSION 2.8.3)
project(moveit_benchmarks)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package()

find_package(Boost REQUIRED COMPONENTS program_options system)
find_package(catkin COMPONENTS
    moveit_core
    moveit_ros_planning_interface
)

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)
if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
    message(FATAL_ERROR "Using old verison of yaml-cpp (< 0.5.0) API. Newer version required.")
endif ()

include_directories(
    "include/"
    ${Boost_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${YamlCpp_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

# MoveIt plugin.
if (MOVEIT_CORE_FOUND)
    add_executable(run_moveit_benchmark
        src/MoveItModule.cpp
    )
    target_link_libraries(run_moveit_benchmark
        ${catkin_LIBRARIES}
        ${YamlCpp_LIBRARIES}
    )
    install(TARGETS run_moveit_benchmark
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif (MOVEIT_CORE_FOUND)