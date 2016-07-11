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
    double result = 0.0;

    // Parse the input stream to obtain which method to call
    std::string inp_arg;
    sin >> inp_arg;

    // Check if one word or two - Assume robot has DOF 
    size_t blank_count = std::count(inp_arg.begin(),inp_arg.end(),' ');

    // Assume method name <blank> kinbody <optional-blank> object
    size_t first_blank_pos = inp_arg.find(" ");
    std::string method_name = inp_arg.substr(0,first_blank_pos);

    if (blank_count == 1)
    {
        // Get body name
        std::string body_name = inp_arg.substr(first_blank_pos+1);
        OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);
        std::cout<<method_name<<" - "<<body_name<<std::endl;

        if (method_name == "CheckCollision")
        {
            start = std::chrono::high_resolution_clock::now();
            checker_ptr -> CheckCollision(pbody);
            end = std::chrono::high_resolution_clock::now();
        }
        else
        {
            //Must be CheckSelfCollision
            start = std::chrono::high_resolution_clock::now();
            checker_ptr -> CheckSelfCollision(pbody);
            end = std::chrono::high_resolution_clock::now();
        }

    }
    else if (blank_count == 2)
    {
        // Must be 2 - Get body and object
        size_t second_blank_pos = inp_arg.find(" ",first_blank_pos+1);
        std::string body_name = inp_arg.substr(first_blank_pos+1,second_blank_pos - first_blank_pos - 1);
        OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);

        std::string object_name = inp_arg.substr(second_blank_pos+1);
        OpenRAVE::KinBodyConstPtr object = GetEnv() -> GetKinBody(object_name);

        std::cout<<method_name<<" - "<<body_name<<" "<<object_name<<std::endl;

        //Method name must be CheckCollision
        start = std::chrono::high_resolution_clock::now();
        checker_ptr -> CheckCollision(pbody,object);
        end = std::chrono::high_resolution_clock::now();
    }
    else
    {
        throw std::runtime_error("Invalid input stream specification!");
    }

    // Report difference
    std::chrono::duration<double> elapsed_seconds = end-start;
    result = elapsed_seconds.count();

    sout<<result;

    return true;

}