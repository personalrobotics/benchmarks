#!/usr/bin/env python

# Thanks to Chris Dellin for this nice hack. :-)
#if ord(os.environ.get('ROS_DISTRO', 'hydro')[0]) <= ord('f'):
package_name='or_benchmarks'
import roslib; roslib.load_manifest(package_name)
import prpy; prpy.dependency_manager.export(package_name)

import os
from rospkg import RosPack
from run_collision_benchmark import *
from analyze import *

def run_self_collision(engines):
    # First set the file for the benchmark tests
    benchmark_file = 'self_benchmark.test'

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
    
    benchmark_file = 'env_benchmark.test'

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
    parser.add_argument('--engines', type=str, nargs='+', default=['ode','pqp','fcl'],
                        help="The collision checkers to test")
    args = parser.parse_args()


    # Self collision
    self_collision_data = run_self_collision(args.engines)
    outfile = 'self_collision.png'
    analyze(self_collision_data, title='Self Collision', outfile=outfile)
    print 'Saved self collision results to %s' % outfile

    # Empty envirnment
    empty_collision_data = run_environment_collision(args.engines)
    outfile = 'empty_env_collision.png'
    analyze(empty_collision_data, title='Empty Environment Collisions', outfile=outfile)
    print 'Saved empty environment results to %s' % outfile

    # Intel kitceen
    rp = RosPack()
    kitchen_env = os.path.join(rp.get_path('pr_ordata'), 'ordata', 'environments', 'intelkitchen.env.xml')
    outfile = 'intel_collision.png'
    kitchen_collision_data = run_environment_collision(args.engines, kitchen_env, 'kitchen')
    analyze(kitchen_collision_data, title='Intel Kitchen Environment', outfile=outfile)
    print 'Saved intel kitchen results to %s' % outfile

    # pr kitchen
    rp = RosPack()
    kitchen_env = os.path.join(rp.get_path('pr_kitchen'),'ordata','pr_kitchen.env.xml')
    kitchen_collision_data = run_environment_collision(args.engines, kitchen_env, 'kitchen')
    outfile = 'prkitchen_collision.png'
    analyze(kitchen_collision_data, title='PR Kitchen Environment', outfile=outfile)
    print 'Saved pr_kitchen results to %s' % outfile




