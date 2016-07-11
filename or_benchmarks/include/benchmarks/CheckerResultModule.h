#ifndef CHECKER_RESULT_MODULE_H
#define CHECKER_RESULT_MODULE_H

#include <openrave/openrave.h>

namespace benchmarks {

class CheckerResultModule : public OpenRAVE::ModuleBase {

public:
    CheckerResultModule(OpenRAVE::EnvironmentBasePtr env);

    virtual ~CheckerResultModule();

private:

    bool EvaluateCheck(std::ostream &out, std::istream &in);
};

}

#endif // CHECKER_RESULT_MODULE_H