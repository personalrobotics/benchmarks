#!/usr/bin/env python
import logging, numpy, os

logger = logging.getLogger('or_benchmarks')

from rospkg import RosPack
ros_pack = RosPack()
package_path = ros_pack.get_path('or_benchmarks')

def get_planner(planner_name):
    """
    Convert a planner name to a planner
    """
    if planner_name == 'cbirrt':
        from prpy.planning import CBiRRTPlanner
        planner = CBiRRTPlanner()
    elif planner_name == 'chomp':
        from prpy.planning import CHOMPPlanner
        planner = CHOMPPlanner()
    elif planner_name == 'snap':
        from prpy.planning import SnapPlanner
        planner = SnapPlanner()
    elif planner_name == 'trajopt':
        from or_trajopt import TrajoptPlanner
        planner = TrajoptPlanner()
    else:
        raise Exception('Unrecognized planner name')

    return planner

def run_planning_benchmark(env, robot, planner,
                           env_serialization_file=None, 
                           benchmark_file=None,
                           num_samples=10000,
                           savefile=None):
    """
    @param env The OpenRAVE environment
    @param robot The OpenRAVE robot
    @param planner The planner to benchmark
    @param env_serialization_file The file containing the json serialized environment
      (generated via prpy.serialization.serialize_environment_file)
      If None, the current state of the environment is used
    @param benchmark_file A file containing pairs of configurations to plan between
      If None, random poses are generated
    @param num_samples If benchmark_file is None, the number of random configurations
      to generate
    @param savefile The file to save the configuration pairs used for benchmarking
    """
    benchmark_configurations = None
    if benchmark_file is not None:
        import yaml
        with open(benchmark_file, 'r') as f:
            benchmark_data = yaml.load(f.read())
        benchmark_configurations = benchmark_data['configurations']
        env_serialization_file = os.path.join(package_path, 'datasets', benchmark_data['env_file'])

    if env_serialization_file is not None:
        import json
        with open(env_serialization_file, 'r') as f:
            serialized_env = json.loads(f.read())
        
        from prpy.serialization import deserialize_environment
        deserialize_environment(serialized_env, env=env, reuse_bodies=[ robot ])

    if benchmark_configurations is None:
        logger.info('Generating %d valid configuration pairs...', num_samples)
        import random
        benchmark_configurations = []
        valid_samples = 0
        lower, upper = robot.GetDOFLimits()
        while valid_samples < num_samples:
            # Select a random arm
            with env:
                manipulators = robot.GetManipulators()
                m = random.sample(manipulators, 1)[0]
                robot.SetActiveManipulator(m)
                robot.SetActiveDOFs(m.GetArmIndices())
                l, u = robot.GetActiveDOFLimits()

            # sample random start - full configuration
            start_config = [random.uniform(lower[i], upper[i]) for i in range(len(lower))]
            with env:
                robot.SetDOFValues(start_config)
            if env.CheckCollision(robot) or robot.CheckSelfCollision():
                continue

            # sample a random goal for just the active dofs
            end_config = [random.uniform(l[i], u[i]) for i in range(len(l))]
            with env:
                robot.SetActiveDOFValues(end_config)
            if env.CheckCollision(robot) or robot.CheckSelfCollision():
                continue

            v = { 'manipulator': m.GetName(),
                  'start': start_config,
                  'end': end_config }
            benchmark_configurations.append(v)
            valid_samples += 1
        logger.info('Done')

    from prpy.util import Timer
    from prpy.planning import PlanningError
    total_time = 0
    valid_plans = 0
    planning_method = getattr(planner, 'PlanToConfiguration')

    for config in benchmark_configurations:
        manip = robot.GetManipulator(config['manipulator'])
        with env:
            robot.SetActiveManipulator(manip)
            robot.SetActiveDOFs(manip.GetArmIndices())
            robot.SetDOFValues(config['start']) # start defines full configuration
        with Timer() as timer:
            try:
                found_solution=False
                planning_method(robot, config['end'])
                found_solution=True
                valid_plans += 1
            except PlanningError as ignored:
                pass
        config['elapsed_ms'] = timer.get_duration()/1000.0
        config['found_solution'] = found_solution
        total_time += config['elapsed_ms']
    
    if savefile is not None:
        benchmark_data = {'env_file': os.path.basename(env_serialization_file),
                          'configurations': benchmark_configurations}
        import yaml
        with open(savefile, 'w') as f:
            f.write(yaml.dump(benchmark_data))
        logger.info('Benchmark data saved to file %s', savefile)
        
    num_tests = len(benchmark_configurations)
    logger.info('Planner solved %d of %d problems', valid_plans, num_tests)
    logger.info('Average plan time: %0.3f seconds (%d tests)', 
                total_time/float(num_tests), num_tests)

if __name__ == '__main__':
    
    import argparse
    parser = argparse.ArgumentParser(description="Run the planning benchmark test")
    parser.add_argument("--outfile", type=str, default=None,
                        help="The output file to save results in, if none results are not saved")
    parser.add_argument("--test", type=str, default=None,
                        help="The list of configurations to check, this should be in the form of a"
                        " results file from a previous run. If None, random configurations"
                        " will be sampled")
    parser.add_argument("--env", type=str, required=None,
                        help="The serialized environment to load - only used if test is none")
    parser.add_argument("--random", type=int, default=10000,
                        help="The number of random configurations to check. This will be ignored"
                        " if the test parameter is set.")
    parser.add_argument("--planner", type=str, default="cbirrt", 
                        choices=['cbirrt', 'chomp', 'snap', 'trajopt'],
                        help="The planner to benchmark")
    parser.add_argument("--cc", type=str, default="ode",
                        help="The collision checker to use")
    args = parser.parse_args()

    import herbpy
    env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker') 

    from prpy.logger import initialize_logging
    initialize_logging()

    # Set the collision checker
    import openravepy
    cc = openravepy.RaveCreateCollisionChecker(env, args.cc)
    if(args.cc == 'sdf'):
        cc.SendCommand('SetSubChecker ode')
    cc.SetCollisionOptions(0)
    if cc is None:
        raise Exception('Invalid collision engine. Failing.')
    env.SetCollisionChecker(cc)

    planner = get_planner(planner_name=args.planner)
    run_planning_benchmark(env, robot, planner,
                           env_serialization_file=args.env, 
                           benchmark_file=args.test,
                           num_samples=args.random,
                           savefile=args.outfile)

    import IPython; IPython.embed()
