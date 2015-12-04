def get_planner(planner_name, **kwargs):
    """
    Convert a planner name to a planner
    """
    if planner_name == 'cbirrt':
        from prpy.planning import CBiRRTPlanner
        planner = CBiRRTPlanner(**kwargs)
    elif planner_name == 'chomp':
        from prpy.planning import CHOMPPlanner
        planner = CHOMPPlanner(**kwargs)
    elif planner_name == 'snap':
        from prpy.planning import SnapPlanner
        planner = SnapPlanner(**kwargs)
    elif planner_name == 'trajopt':
        from or_trajopt import TrajoptPlanner
        planner = TrajoptPlanner(**kwargs)
    else:
        from ..benchmark_utils import BenchmarkException
        raise BenchmarkException('Unrecognized planner name %s', planner_name)

    return planner

class BenchmarkPlannerMetadata(object):

    def __init__(self, planner_name=None, planner_parameters=None):
        """
        @param planner_name The name of the planner (string - see get_planner)
        @param planner_parameters A dictionary of parameters for the planner
        """
        self.planner_name = planner_name
        self.planner_parameters = planner_parameters if planner_parameters is not None else dict()

    def to_yaml(self):
        """
        Serialize this planner metadata to a yaml dictionary
        """
        import yaml
        p = { 'planner_name', self.planner_name,
              'planner_parameters', yaml.dump(self.planner_parameters) }
        return p

    def from_yaml(self, planner_yaml):
        """
        @param planner_yaml The planner metadata in yaml format
        """
        self.planner_name = planner_yaml.get('planner_name', None)
        planner_parameters = planner_yaml.get('planner_parameters', None)
        if planner_parameters is not None:
            import yaml
            self.planner_parameters = yaml.load(planner_parameters)
        else:
            self.planner_parameters = dict()
