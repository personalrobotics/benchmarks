import logging
logger = logging.getLogger('prpy_benchmarks')

class BenchmarkException(Exception):
    pass

def execute_benchmark(queryfile, plannerfile, env=None, robot=None, outfile=None):
    """
    @param queryfile The file containing the query to execute
    @param plannerfile The file containing metadata about the planner to use
    @param outdir If not none, the file where the result should be written
    """
    import yaml
    from .query import BenchmarkQuery
    query = BenchmarkQuery()

    with open(queryfile, 'r') as f:
        query.from_yaml(yaml.load(f.read()))

    from .planner import BenchmarkPlannerMetadata
    planner_metadata = BenchmarkPlannerMetadata()
    with open(plannerfile, 'r') as f:
        planner_metadata.from_yaml(yaml.load(f.read()))

    # Deserialize the environment - TODO: remove resue bodies
    from prpy.serialization import deserialize_environment
    reuse_bodies = [ robot ] if robot is not None else None
    env = deserialize_environment(query.serialized_env, env=env, reuse_bodies=reuse_bodies)

    # Figure out which planner to use
    from .planner import get_planner
    planner = get_planner(planner_metadata.planner_module,
                          planner_metadata.planner_class_name,
                          **planner_metadata.planner_parameters)

    # Get the planning method from the planner
    planning_method = getattr(planner, query.planning_method)

    # Execute with timing
    from prpy.planning import PlanningError
    from prpy.util import Timer
    with Timer() as timer:
        try:
            success = True
            path = planning_method(robot, *query.args, **query.kwargs)
        except PlanningError as e:
            success = False
    plan_time = timer.get_duration()

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
    
    if outfile is not None:
        with open(outfile, 'w') as f:
            f.write(yaml.dump(result.to_yaml()))
        logger.info('Results written to file %s', outfile)
    
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
