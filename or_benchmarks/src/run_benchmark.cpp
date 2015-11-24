#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/timer.hpp>
#include <google/profiler.h>
#include <openrave/openrave.h>
#include <openrave-core.h>
#include <benchmarks/DataUtils.h>

using boost::format;
using boost::str;

using namespace OpenRAVE;
namespace po = boost::program_options;

int main(int argc, char **argv)
{
    std::string environment_uri;
    std::string robot_name;
    std::string plugin_name;
    size_t num_trials;
    bool self = false;
    bool profile = false;

    // Parse arguments.
    po::options_description desc("Profile OpenRAVE's memory usage.");
    desc.add_options()
        ("num-samples", po::value<size_t>(&num_trials)->default_value(10000),
            "number of samples to run")
        ("self", po::value<bool>(&self)->zero_tokens(),
            "run self-collision checks")
        ("profile", po::value<bool>(&profile)->zero_tokens(),
            "remove objects from environment")
        ("environment_uri",
            po::value<std::string>(&environment_uri)->required(),
            "number of samples to run")
        ("robot", po::value<std::string>(&robot_name)->required(),
            "robot_name")
        ("collision_checker",
            po::value<std::string>(&plugin_name)->required(),
            "collision checker name")
        ("help",
            "print usage information")
        ;

    po::positional_options_description pd;
    pd.add("environment_uri", 1);
    pd.add("robot", 1);
    pd.add("collision_checker", 1);

    po::variables_map vm;
    po::store(
        po::command_line_parser(argc, argv)
            .options(desc)
            .positional(pd).run(),
        vm
    );
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    // Create the OpenRAVE environment.
    RaveInitialize(true);

    EnvironmentBasePtr const env = RaveCreateEnvironment();
    CollisionCheckerBasePtr const collision_checker
            = RaveCreateCollisionChecker(env, plugin_name);
    env->SetCollisionChecker(collision_checker);

    env->StopSimulation();

    // "/usr/share/openrave-0.9/data/wamtest1.env.xml"
    env->Load(environment_uri);
    KinBodyPtr const body = env->GetKinBody(robot_name);

    // Generate random configuations.
    std::vector<OpenRAVE::dReal> lower;
    std::vector<OpenRAVE::dReal> upper;
    body->GetDOFLimits(lower, upper);
	std::vector<std::vector<double> > data;
    data = benchmarks::DataUtils::GenerateRandomConfigurations(
            num_trials, lower, upper);

    //
    RAVELOG_INFO("Running %d collision checks.\n", num_trials);

    boost::timer const timer;
    if (profile) {
        std::string const prof_name = str(
                format("CheckCollision.%s.prof") %  plugin_name);
        RAVELOG_INFO("Writing gperftools information to '%s'.\n",
            prof_name.c_str()
        );

        ProfilerStart(prof_name.c_str());
    }

    size_t num_collision = 0;
    for (size_t i = 0; i < num_trials; ++i) {
        body->SetDOFValues(data[i]);

        bool is_collision;
        if (self) {
            is_collision = body->CheckSelfCollision();
        } else {
            is_collision = env->CheckCollision(body);
        }
        num_collision += !!is_collision;
    }

    if (profile) {
        ProfilerStop();
    }

    double const duration = timer.elapsed();
    RAVELOG_INFO(
        "Ran %d collision checks (%d in collision) in %f seconds (%f checks per second).\n",
        num_trials, num_collision, duration, num_trials / duration
    );


    return 0;
}
