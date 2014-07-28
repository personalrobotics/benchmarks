#include <fstream>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/timer.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

struct SelfCollisionData {
    std::vector<double> pt;
    bool collision;
};

static double const max_duration = 5.0;

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "error: Incorrect number of arguments.\n"
                     "usage: " << argv[0] << " /path/to/poses.yaml\n"
                  << std::endl;
        return 1;
    }
    std::string const input_path = argv[1];

    ros::init(argc, argv, "moveit_benchmark", ros::init_options::AnonymousName);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    planning_scene::PlanningScene planning_scene(kinematic_model);
    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();

    // Load poses from disk.
    std::vector<std::string> dof_names;
    std::vector<SelfCollisionData> poses;
    {
        std::ifstream in_stream(input_path.c_str(), std::ios::binary);
        YAML::Parser parser(in_stream);
        YAML::Node doc;
        parser.GetNextDocument(doc);

        doc["names"] >> dof_names;

        for (int i = 0; i < doc["data"].size(); ++i) {
            SelfCollisionData entry;
            doc["data"][i]["pose"] >> entry.pt;
            doc["data"][i]["collision"] >> entry.collision;
            poses.push_back(entry);
        }
    }
    std::cout << "Loaded " << poses.size() << " poses from '" << input_path << "'." << std::endl;

    int num_checks = 50000;
    int num_collisions = 0;
    double duration = 0.0;
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    BOOST_FOREACH (SelfCollisionData const &query, poses) {
        BOOST_ASSERT(query.pt.size() == dof_names.size());
        for (int i = 0; i < dof_names.size(); ++i) {
            current_state.setJointPositions(dof_names[i], &query.pt[i]);
        }

        boost::timer t;
        collision_result.clear();
        planning_scene.checkSelfCollision(collision_request, collision_result);
        duration += t.elapsed();

        if (collision_result.collision) {
            num_collisions++;
        }
    }

    std::cout << "Total self-collision checks: " << num_checks << '\n'
              << "Total collisions found: " << num_collisions << '\n'
              << "Total ellapsed time: " << duration << '\n'
              << "Average time per check: " << (duration / num_collisions) << '\n'
              << "Average checks per second: " << (num_collisions / duration) << '\n'
              << std::flush;
    return 0;
}
