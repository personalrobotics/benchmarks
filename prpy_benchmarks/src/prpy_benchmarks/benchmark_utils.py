import logging
logger = logging.getLogger('prpy_benchmarks')

class BenchmarkException(Exception):
    pass

def execute_benchmark(queryfile, plannerfile, log_collision_checks=False, env=None, robot=None, outfile=None):
    """
    @param queryfile The file containing the query to execute
    @param plannerfile The file containing metadata about the planner to use
    @param log_collision_checks Whether or not to log collision checks
    @param env The env to run the query against (if None a new env 
      is created)
    @param robot The robot to use (if None, the robot is loaded as 
      part of deserialization)
    @param outdir If not none, the file where the result should be written
    """
    import yaml
    import openravepy

    # TODO Currently for a single planner only
    # TODO Responsibility for doing for multiple planners with higher level user

    # Load the query
    from .query import BenchmarkQuery
    query = BenchmarkQuery()
    with open(queryfile, 'r') as f:
        query.from_yaml(yaml.load(f.read()), env=env, robot=robot)

    # Load the planner metadata
    from .planner import BenchmarkPlannerMetadata
    planner_metadata = BenchmarkPlannerMetadata()
    with open(plannerfile, 'r') as f:
        planner_metadata.from_yaml(yaml.load(f.read()))

    # Figure out which planner to use
    from .planner import get_planner
    planner = get_planner(planner_metadata.planner_module,
                          planner_metadata.planner_class_name,
                          **planner_metadata.planner_parameters)

    #Set to stubchecker if collisions to be logged
    if log_collision_checks:
        stubchecker = openravepy.RaveCreateCollisionChecker(env,'stubchecker')
        env.SetCollisionChecker(stubchecker)

    # Get the planning method from the planner
    planning_method = getattr(planner, query.planning_method)

    # Execute with timing
    from prpy.planning import PlanningError
    from prpy.util import Timer
    with Timer() as timer:
        try:
            success = True
            path = planning_method(*query.args, **query.kwargs)
        except PlanningError as e:
            success = False
            path = None
    plan_time = timer.get_duration()

    if not log_collision_checks:
        # Create a result object
        from .result import BenchmarkResult
        from os.path import basename

        result = BenchmarkResult(queryfile=basename(queryfile), 
                                 query_md5=to_md5(queryfile),
                                 plannerfile=basename(plannerfile),
                                 planner_data_md5=to_md5(plannerfile),
                                 time=plan_time, 
                                 success=success, 
                                 path=path)

        # Save to yaml
        if outfile is not None:
            with open(outfile, 'w') as f:
                f.write(yaml.dump(result.to_yaml()))
            logger.info('Results written to file %s', outfile)

    else:
        from .result import BenchmarkCollisionResult
        import json

        # Request collision check logs from stubchecker
        check_info = stubchecker.SendCommand('GetLogInfo')
        check_info_dict = json.loads(check_info)
        stubchecker.SendCommand('Reset')

        # Save only the relevant DOF values used for self+env checks
        relevant_dof_list = get_relevant_DOF_list(check_info_dict)

        result = BenchmarkCollisionResult(environment=env,
                                          collision_log=relevant_dof_list)

        # Save to json
        if outfile is not None:
            with open(outfile,'w') as f:
                json.dump(result.to_dict(),f)
            logger.info('Results written to file %s',outfile)
    
    return result


def execute_collisioncheck_benchmark(engine,robot_name,collresultfile=None,
                                     option_dofs='active',env_first = True, 
                                     env_first_outfile=None, self_first_outfile=None): 
    """
    Expects a special format that has only the list of relevant DOFs done for
    self and env collisions. Default order is env check first, then self check,
    for both logging and evaluating
    @param engine The collision checker engine to use (ODE/PQP/FCL)
    @param robot_name The name of the robot to check for 
    @param collresultfile The name of the file that has the environment and logged checks 
    @param option_dofs Whether to check with Active DOFs or all DOFs
    @param outfile The file to save results too
    """
    import json
    import openravepy
    import atexit

    if collresultfile is None:
        raise Exception('No valid collision log file specified')

    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    atexit.register(openravepy.RaveDestroy)
    env = openravepy.Environment()
    atexit.register(env.Destroy)

    with open(collresultfile,'r') as fin:
        collresultdict = json.load(fin)
        collresult = BenchmarkCollisionResult()
        collresult.from_dict(collresultdict,env)

    # Get robot
    robot = collresult.environment.GetRobot(robot_name)

    # Lists for noting elapsed time and check result
    time_list = list()
    result_list = list()

    # Lock openrave environment
    with collresult.environment:

        # Create module to evaluate check time
        checkermodule = openravepy.RaveCreateModule(env,'checkerresultmodule')

        # Set collision checker to evaluate
        cc = openravepy.RaveCreateCollisionChecker(env,engine)
        if option_dofs == 'active':
            cc.SetCollisionOptions(openravepy.CollisionOptions.ActiveDOFs)
        else:
            cc.SetCollionOptions(0)

        if cc is None:
            raise Exception('Invalid collision engine. Failing.')
        env.SetCollisionchecker(cc)

        for dof_vals in collresult.collision_log:

            robot.SetDOFValues(dof_vals)
            single_check_time = 0.0

            # Specify env and self check methods
            standalone_checklog_dict = {'methodname': 'CheckStandaloneSelfCollision',
                                'body' : robot_name}
            body_env_checklog_dict = {'methodname' : 'CheckCollision_body_env',
                                'body' : robot_name}

            check_order = [body_env_checklog_dict,standalone_checklog_dict]

            for spec_dict in check_order:

                check_specs_str = json.dumps(spec_dict)
                string_command = 'EvaluateCheck '+check_specs_str
                check_and_time_str = module.SendCommand(string_command)
                check_and_time = check_and_time_str.split(' ')

                single_check_result = False
                if int(check_and_time[0]) == 1:
                    single_check_result = True
                single_check_time = float(check_time[1])

                # Update performance_dict
                time_list.append(single_check_time)
                result_list.append(single_check_result)


    # Save depending on env first or self first
    if env_first:
        env_times, self_times = split_by_env_first(time_list,result_list)
    else:
        self_times, env_times = split_by_self_first(time_list,result_list)

    # Separate dictionaries for env and self first
    env_performance_dict = {'checks':len(env_times),'elapsed_sec':sum(env_times),'data':{'times':env_times}}
    self_performance_dict = {'checks':len(self_times),'elapsed_sec':sum(self_times),'data':{'times':self_times}}

    # Save to file
    if outfile is not None:
        with open(env_first_outfile,'w') as fout:
            json.dump(env_performance_dict,fout)
        with open(self_first_outfile,'w') as fout:
            json.dump(self_performance_dict,fout)

        logger.info('Collision Benchmarking Results written to files')


def get_relevant_DOF_list(check_info_dict):

    """
    From the output of stub_checker, get the unique sets of
    DOFs that are queried for self and env checks
    @ param check_info_dict The dictionary of collision check information
        obtained from a single run of or_stub_checker
    """
    n_keys = len(check_info_dict.keys())
    prev_method_name = ''
    relevant_DOF_list = list()

    for i in range(1,n_keys+1):

        check_key = str(i)
        if check_key in check_info_dict.keys():
            
            record = check_info_dict[key]
            method_name = record['methodname']
            
            if method_name == 'CheckStandaloneSelfCollision':
                if prev_method_name == 'CheckCollision_body_env':

                    # Relevant DOF set; record in list
                    relevant_DOF_list.append(check_info_dict[key]['dofvals'])
        
            prev_method_name = method_name

    return relevant_DOF_list


def split_by_env_first(time_list,result_list):
    """
    Split the elapsed time for checks and the result of the check,
    assuming the environment check is first
    @param time_list The list of elapsed times
    @param result_list The corresponding list of check results
    """

    self_times = list()
    env_times = list()

    n_vals = len(result_list)

    for i in range(n_vals/2):

        # Env time is every (2*i)th and self is (2*i+1)th
        env_times.append(time_list[2*i])
        if res_list[2*i] == False:
            # Needed to check self too to verify
            self_times.append(time_list[2*i+1])

    return env_times, self_times


def split_by_self_first(time_list,result_list):
    """
    Split the elapsed time for checks and the result of the check,
    assuming the self-collision check is first
    @param time_list The list of elapsed times
    @param result_list The corresponding list of check results
    """

    self_times = list()
    env_times = list()

    n_vals = len(result_list)

    for i in range(n_vals/2):

        # Env time is every (2*i)th and self is (2*i+1)th
        self_times.append(time_list[2*i]+1)
        if res_list[2*i+1] == False:
            # Needed to check env too to verify
            env_times.append(time_list[2*i])

    return self_times, env_times


def to_md5(filename):
    """
    Read in a file and compute md5 checksum
    """
    import hashlib
    hash = hashlib.md5()
    with open(filename, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash.update(chunk)
    return hash.hexdigest()
