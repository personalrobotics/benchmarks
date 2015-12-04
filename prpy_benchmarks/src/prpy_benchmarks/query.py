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
        import prpy

        if self.serialized_env is None:
            raise BenchmarkException('The query has no serialized environment')
        if self.planning_method is None:
            raise BenchmarkException('The query has no planning method defined')

        query = {}
        query['serialized_env'] = self.serialized_env
        query['args'] = prpy.serialize(self.args)
        query['kwargs'] = prpy.serialize(self.kwargs)
        query['planning_method'] = prpy.serialize(self.planning_method)
        return query

    def from_yaml(self, query_yaml):
        """
        @param query_yaml The query in yaml format
        """
        self.planning_method=query_yaml.get('planning_method', None)
        self.serialized_env=query_yaml.get('serialized_env', None)
        self.args=query_yaml.get('args', list())
        self.kwargs=query_yaml.get('kwargs', dict())
