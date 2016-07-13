#include <benchmarks/CheckerResultModule.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <string>
#include <exception>
#include <algorithm>

using namespace benchmarks;

CheckerResultModule::CheckerResultModule(OpenRAVE::EnvironmentBasePtr env)
    : ModuleBase(env) {

    RegisterCommand("EvaluateCheck", boost::bind(&CheckerResultModule::EvaluateCheck, this, _1, _2),
                    "Time the collision check");
}

CheckerResultModule::~CheckerResultModule() {

}

bool CheckerResultModule::EvaluateCheck(std::ostream &sout, std::istream &sin) {

    // Obtain checker for environment
    OpenRAVE::CollisionCheckerBasePtr checker_ptr = GetEnv() -> GetCollisionChecker();

    if (!checker_ptr)
    {
        throw std::runtime_error("Environment does not have valid collision checker!");
    }

    // Result to output
    std::chrono::time_point<std::chrono::high_resolution_clock> start,end;
    double time_result = 0.0;
    bool check_result = false;

    // Parse the input stream to obtain which method to call
    std::string method_name;
    sin >> method_name;

    std::string notrim_inp_arg;
    std::getline(sin,notrim_inp_arg);
    

    //Trim leading whitespaces
    std::string inp_arg = notrim_inp_arg.substr(notrim_inp_arg.find_first_not_of(' '));
    std::cout<<"Input argument is "<<inp_arg<<std::endl;

    // Check if one word or two - Assume robot has DOF 
    size_t blank_count = std::count(inp_arg.begin(),inp_arg.end(),' ');

    if (blank_count == 0)
    {
        // Get body name
        std::string body_name(inp_arg);
        OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);
        std::cout<<method_name<<" - "<<body_name<<std::endl;

        if (method_name == "CheckCollision")
        {
            start = std::chrono::high_resolution_clock::now();
            check_result = checker_ptr -> CheckCollision(pbody);
            end = std::chrono::high_resolution_clock::now();
        }
        else
        {
            //Must be CheckSelfCollision
            start = std::chrono::high_resolution_clock::now();
            check_result = checker_ptr -> CheckSelfCollision(pbody);
            end = std::chrono::high_resolution_clock::now();
        }

    }
    else if (blank_count == 1)
    {
        // Must be 2 - Get body and object
        size_t first_blank_pos = inp_arg.find(" ");
        std::string body_name = inp_arg.substr(0,first_blank_pos);
        OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);

        std::string object_name = inp_arg.substr(first_blank_pos+1);
        OpenRAVE::KinBodyConstPtr object = GetEnv() -> GetKinBody(object_name);

        std::cout<<method_name<<" - "<<body_name<<" "<<object_name<<std::endl;

        //Method name must be CheckCollision
        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(pbody,object);
        end = std::chrono::high_resolution_clock::now();
    }
    else
    {
        throw std::runtime_error("Invalid input stream specification!");
    }

    // Report difference
    std::chrono::duration<double> elapsed_seconds = end-start;
    time_result = elapsed_seconds.count();

    sout<<check_result<<" "<<time_result;

    return true;

}