class BenchmarkQueryResult(object):

    def __init__(self, planner_data=None, query_data=None,
                 time=0.0, success=False, path=None):
        """
        @param planner_data The BenchmarkPlannerMetadata object used to generate
          the planner used by the query
        @param query_data The BenchmarkQuery object used to generate this result
        @param time The time to generate this result
        @param success True if this query successfully executed
        @param path The output of the planning call
        """
        self.planner_data=planner_data
        self.query_data=query_data
        self.time=time
        self.success=success
        self.path=path

    def to_yaml(self):
        """
        Serialize this result to a yaml dictionary
        """
        if self.planner_data is None:
            raise BenchmarkException('No planner data associated with this result')
        if self.query_data is None:
            raise BenchmarkException('No query data associated with this result')

        path_data = None
        if self.path is not None:
            path_data = self.path.serialize()
        result = {'planner_data': self.planner_data.to_yaml(),
                  'query_data': self.query_data.to_yaml(),
                  'time': self.time,
                  'success': self.success,
                  'path': path_data
              }

        return result

    def from_yaml(self, result_yaml):
        """
        @param result_yaml The result in yaml format
        """
        pd = result_yaml.get('planner_data', None)
        if pd is not None:
            from ..planner import BenchmarkPlanner
            self.planner_data = BenchmarkPlanner()
            self.planner_data.from_yaml(pd)

        qd = result_yaml.get('query_data', None)
        if qd is not None:
            from ..query import BenchmarkQuery
            self.query_data = BenchmarkQuery()
            self.query_data.from_yaml(qd)
        
        self.time = result_yaml.get('time', 0.0)
        self.success = result_yaml.get('success', False)

        # TODO: Load in the path
