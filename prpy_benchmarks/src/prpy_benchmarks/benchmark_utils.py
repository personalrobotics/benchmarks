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

    print 'Executing query from file: %s' % queryfile
    result = query.execute(planner_metadata, env, robot)

    if outfile is not None:
        with open(outfile, 'w') as f:
            f.write(yaml.dump(result.to_yaml()))
        logger.info('Results written to file %s', outfile)
