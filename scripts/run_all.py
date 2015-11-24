#!/usr/bin/env python
import argparse, os

# Thanks to Chris Dellin for this nice hack. :-)
package_name='or_benchmarks'
if ord(os.environ.get('ROS_DISTRO', 'hydro')[0]) <= ord('f'):
    import roslib; roslib.load_manifest(package_name)
    import prpy; prpy.dependency_manager.export(package_name)

from run_collision_benchmark import *
from analyze import *

from rospkg import RosPack
ros_pack = RosPack()
package_path = ros_pack.get_path(package_name)

def run_self_collision(engines):
    # First set the file for the benchmark tests
    benchmark_file = os.path.join(package_path, 'datasets', 'self_benchmark.test')

    datafiles = []
    for engine in engines:
        outfile = '%s_self.results' % engine
        datafiles.append(outfile)

        run_benchmark('herb', engine, 
                      self_collision = True,
                      testfile = benchmark_file,
                      outfile = outfile)

    return datafiles

def run_environment_collision(engines, env_file=None, test_description='empty'):
    new_args = {}
    
    benchmark_file = os.path.join(package_path, 'datasets', 'env_benchmark.test')

    datafiles = []
    for engine in engines:
        outfile = '%s_%s.results' % (engine, test_description)
        datafiles.append(outfile)
        run_benchmark('herb', engine,
                      envfile = env_file,
                      testfile = benchmark_file,
                      outfile = outfile)

    return datafiles
    

if __name__ == '__main__':    

    parser = argparse.ArgumentParser(description="Run the benchmark tests")
    parser.add_argument('--engines', type=str, nargs='+', default=['sdf','ode','pqp','fcl'],
                        help="The collision checkers to test")
    args = parser.parse_args()

    # Self collision
    self_collision_data = run_self_collision(args.engines)
    out_basename = os.path.join(package_path, 'results', 'self_collision')
    analyze(self_collision_data, title='Self Collision', out_basename=out_basename)
    
    # Empty envirnment
    empty_collision_data = run_environment_collision(args.engines)
    out_basename = os.path.join(package_path, 'results', 'empty_env_collision')
    analyze(empty_collision_data, title='Empty Environment Collisions', out_basename=out_basename)

    # PR kitchen
    from catkin.find_in_workspaces import find_in_workspaces
    kitchen_env = find_in_workspaces(
        search_dirs=['share'],
        project='pr_ordata',
        path='data/kitchen/pr_kitchen.env.xml',
        first_match_only=True)[0]
        
    kitchen_collision_data = run_environment_collision(args.engines, kitchen_env, 'kitchen')
    out_basename = os.path.join(package_path, 'results', 'prkitchen_collision')
    analyze(kitchen_collision_data, title='PR Kitchen Environment', out_basename=out_basename)




