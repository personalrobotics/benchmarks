class BenchmarkResult(object):

    def __init__(self, queryfile=None, query_md5=None,
                 plannerfile=None, planner_data_md5=None,
                 time=0.0, success=False, path=None):
        """
        @param queryfile The filename of the query used to generate this result
           (only basename)
        @parma query_md5 The md5 checksum of the query file
        @param plannerfile The filename of the planner metadata used to generate this
           result (only basename)
        @param planner_data_md5 The md5 checksum of the planner file
        @param time The time to generate this result
        @param success True if this query successfully executed
        @param path The output of the planning call
        """
        self.queryfile=queryfile
        self.query_md5=query_md5
        self.plannerfile=plannerfile
        self.planner_data_md5=planner_data_md5
        self.time=time
        self.success=success
        self.path=path

    def to_yaml(self):
        """
        Serialize this result to a yaml dictionary
        """
        import yaml
        from prpy.serialization import serialize

        if self.plannerfile is None or self.planner_data_md5 is None:
            raise BenchmarkException('No planner data associated with this result')
        if self.queryfile is None or self.query_md5 is None:
            raise BenchmarkException('No query data associated with this result')

        result = {'queryfile': self.queryfile,
                  'query_md5': self.query_md5,
                  'plannerfile': self.plannerfile,
                  'planner_data_md5': self.planner_data_md5,
                  'time': self.time,
                  'success': self.success,
                  'path': serialize(self.path)
              }

        return result

    def from_yaml(self, result_yaml):
        """
        @param result_yaml The result in yaml format
        """
        self.queryfile = result_yaml.get('queryfile', None)
        self.query_md5 = result_yaml.get('query_md5', None)
        self.plannerfile = result_yaml.get('plannerfile', None)
        self.planner_data_md5 = result_yaml.get('planner_data_md5', None)

        self.time = result_yaml.get('time', 0.0)
        self.success = result_yaml.get('success', False)

        # TODO: Load in the path

