#include <iostream>
#include <boost/timer.hpp>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

static double const max_duration = 5.0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_benchmark", ros::init_options::AnonymousName);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    planning_scene::PlanningScene planning_scene(kinematic_model);
    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    boost::timer t;
    int num_checks = 0;
    while (t.elapsed() <= max_duration) {
        current_state.setToRandomPositions();
        collision_result.clear();
        planning_scene.checkSelfCollision(collision_request, collision_result);
        num_checks++;
    }
    double const actual_duration = t.elapsed();

    std::cout << "Checked self-collision " << num_checks << " in "
              << actual_duration << " seconds ("
              << (actual_duration / num_checks) << " per second)."
              << std::endl;
    return 0;
}
