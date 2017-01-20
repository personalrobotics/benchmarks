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

        result = BenchmarkCollisionResult(environment=env,
                                          collision_log=check_info_dict)

        # Save to json
        if outfile is not None:
            with open(outfile,'w') as f:
                json.dump(result.to_dict(),f)
            logger.info('Results written to file %s',outfile)
    
    return result

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
