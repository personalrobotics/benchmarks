#include <iostream>
#include <fstream>
#include <unistd.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/assert.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <openrave/openrave.h>
#include <openrave-core.h>

size_t const num_trials = 10000;
std::string const kinbody_uri = "/opt/pr/pr_ordata/ordata/objects/household/fuze_bottle.kinbody.xml";

using namespace OpenRAVE;
namespace po = boost::program_options;
using boost::iends_with;

static unsigned long get_memory_usage()
{
    static int const kTokenNumber = 24;

    std::ifstream is("/proc/self/stat");

    std::string token;
    for (int itoken = 0; itoken < kTokenNumber; ++itoken) {
        is >> token;
        BOOST_ASSERT(is.good());
    }

    return boost::lexical_cast<unsigned long>(token);
}

static double to_MB(unsigned long const &num_pages)
{
    int const bytes_per_page = getpagesize();
    return static_cast<double>(num_pages * bytes_per_page) / 1e6;
}

int main(int argc, char **argv)
{
    size_t num_trials;
    std::string kinbody_uri;
    bool do_remove = false;
    bool do_destroy = false;

    // Parse arguments.
    po::options_description desc("Profile OpenRAVE's memory usage.");
    desc.add_options()
        ("num-trials,n", po::value<size_t>(&num_trials)->default_value(100),
            "number of objects to create")
        ("remove", po::value<bool>(&do_remove)->zero_tokens(),
            "remove objects from environment")
        ("destroy", po::value<bool>(&do_destroy)->zero_tokens(),
            "destroy objects after removing them")
        ("kinbody", po::value<std::string>(&kinbody_uri),
            "path to KinBody XML file")
        ("help",
            "print usage information")
        ;

    po::positional_options_description pd;
    pd.add("kinbody", 1);

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
    } else if (do_destroy && !do_remove) {
        std::cerr << "Error: Cannot --destroy without --remove." << std::endl;
        return 1;
    }
    // Setup the OpenRAVE environment.
    OpenRAVE::RaveInitialize(true);
    OpenRAVE::RaveSetDebugLevel(Level_Error);
    EnvironmentBasePtr const env = RaveCreateEnvironment();

    // Count the number of meshes in the model.
    int num_meshes_iv = 0;
    int num_meshes_other = 0;
    {
        KinBodyPtr const body = env->ReadKinBodyURI(KinBodyPtr(), kinbody_uri);
        if (!body) {
            std::cerr << "Error: Unable to load KinBody from '"
                      << kinbody_uri << "'." << std::endl;
            return 1;
        }

        BOOST_FOREACH (KinBody::LinkPtr const &link, body->GetLinks()) {
            BOOST_FOREACH (KinBody::Link::GeometryPtr const &geom,  link->GetGeometries()) {
                std::string const mesh_path = geom->GetInfo()._filenamecollision;

                if (iends_with(mesh_path, ".iv")) {
                    num_meshes_iv++;
                } else if (!mesh_path.empty()) {
                    num_meshes_other++;
                }
            }
        }
        env->Remove(body);
    }

    // Run the experiment.
    int const bytes_per_page = getpagesize();
    unsigned long const memory_before = get_memory_usage();

    std::cout << "# Trials: " << num_trials
              << " (Remove? " << do_remove << ","
              << " Destroy? " << do_destroy << ")\n"
              << "KinBody: " << kinbody_uri << "\n"
              << "Meshes: " << num_meshes_iv << " IV, "
                            << num_meshes_other << " other\n"
              << "Memory (before): " << to_MB(memory_before) << " MB"
              << std::endl;

    for (size_t i = 0; i < num_trials; ++i) {
        // Add the object.
        EnvironmentMutex::scoped_lock lock(env->GetMutex()); 
        KinBodyPtr const body = env->ReadKinBodyURI(KinBodyPtr(), kinbody_uri);
        BOOST_ASSERT(body);

        body->SetName("body");
        env->Add(body, true);
        lock.unlock();

        // Remove the object.
        if (do_remove) {
            env->Remove(body);
            if (do_destroy) {
                body->Destroy();
            }
        }
    }

    unsigned long const memory_after = get_memory_usage();
    BOOST_ASSERT(memory_after >= memory_before);
    std::cout << "Memory (after): " << to_MB(memory_after) << " MB\n"
              << "Leaked Memory (total): " << to_MB(memory_after - memory_before) << " MB\n"
              << "Leaked Memory (per object): " << to_MB(memory_after - memory_before) / num_trials << " MB"
              << std::endl;
    return 0;
}
