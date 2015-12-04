class BenchmarkQuery(object):
        
    def __init__(self, planning_method=None, env=None, args=None, kwargs=None):
        """
        @param planning_method The planning method called by this query (string)
        @param env The OpenRAVE environment to run the query against
        @param args A list of arguments to pass to the planning method
        @param kwargs A dictionary of keyword arguments to pass to the planning method
        """
        from prpy.serialization import serialize_environment
        self.planning_method = planning_method
        self.serialized_env = serialize_environment(env) if env is not None else env
        self.args = args if args is not None else list()
        self.kwargs = kwargs if kwargs is not None else dict()

    def to_yaml(self):
        """
        Serialize this query to a yaml dictionary
        """
        if self.serialized_env is None:
            raise BenchmarkException('The query has no serialized environment')
        if self.planning_method is None:
            raise BenchmarkException('The query has no planning method defined')

        query = {}
        query['serialized_env'] = self.serialized_env
        query['args'] = self.args
        query['kwargs'] = self.kwargs
        query['planning_method'] = self.planning_method
        return query

    def from_yaml(self, query_yaml):
        """
        @param query_yaml The query in yaml format
        """
        self.planning_method=query_yaml.get('planning_method', None)
        self.serialized_env=query_yaml.get('serialized_env', None)
        self.args=query_yaml.get('args', list())
        self.kwargs=query_yaml.get('kwargs', dict())

    def execute(self, planner_metadata, env=None, robot=None):
        """
        @param planner_metadata The BenchmarkPlannerMetadata object contiaing
          metdata describing the planner to use
        @param env The environment to run the benchmark in
        @param robot The robot to run the benchmark against
        Execute the query on the given environment
        """
        # Deserialize the environment so its at the correct configuration
        #  for the start of the planning call
        from prpy.serialization import deserialize_environment
        reuse_bodies = [robot] if robot is not None else None
        deserialize_environment(self.serialized_env, env=env, reuse_bodies=reuse_bodies)

        # Figure out which planner to use
        from .planner import get_planner
        planner = get_planner(planner_metadata.planner_name, **planner_metadata.planner_parameters)

        # Get the planning method from the planner
        planning_method = getattr(planner, self.planning_method)

        # Execute with timing
        from prpy.planning import PlanningError
        from prpy.util import Timer
        with Timer() as timer:
            try:
                success = True
                path = planning_method(robot, *self.args, **self.kwargs)
            except PlanningError as e:
                success = False
        plan_time = timer.get_duration()

        # Create a result object
        from .result import BenchmarkQueryResult
        result = BenchmarkQueryResult(planner_metadata, self, plan_time, success, path)
        return result
