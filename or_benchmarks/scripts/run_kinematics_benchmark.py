#!/usr/bin/env python
import argparse, herbpy, openravepy, os

# Thanks to Chris Dellin for this nice hack. :-)
if ord(os.environ.get('ROS_DISTRO', 'hydro')[0]) <= ord('f'):
    package_name='or_benchmarks'
    import roslib; roslib.load_manifest(package_name)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Run the benchmark tests")
    parser.add_argument("--type", type=str, default="fk", choices=['fk', 'jacobian'],
                        help="The type of benchmark to run (fk or jacobian)")
    parser.add_argument("--outfile", type=str, default=None,
                        help="The output file to save results in, if none results are not saved")
    parser.add_argument("--env", type=str, default=None,
                        help="The environment to load")
    parser.add_argument("--robot", type=str, default="herb",
                        help="The robot to use for the FK test")
    parser.add_argument("--manip", type=str, default="right",
                        help="The manipulator to use for the FK test")
    parser.add_argument("--random", type=int, default=50000,
                        help="The number of random poses to check. This will be ignored if the test parameters is set.")
    parser.add_argument("--viewer", type=str, default=None,
                        help="The viewer to attach to the environment")


    args = parser.parse_args()

    # Load the environment
    if args.robot == 'herb':
        env, robot = herbpy.initialize(sim=True, attach_viewer=args.viewer)
    else:
        env = openravepy.Environment()

    if args.env:
        env.Load(args.env)
        
    # Verify the kinbody is in the environment
    body = env.GetKinBody(args.robot)
    if body is None:
        raise Exception('No robot with name %s in environment. Failing.' % args.robot)

    # Load the openrave module
    try:
        module = openravepy.RaveCreateModule(env, 'kinematicbenchmarks')
    except openravepy.openrave_exception:
        raise Exception('Unable to load CollisionCheckingBenchmark module. Check your OPENRAVE_PLUGINS environment variable.')

    # Generate the parameters
    params = {}
    params['robot'] = str(body.GetName())
    params['manip'] = str(args.manip)
    params['random'] = args.random
    if args.outfile is not None:
        params['outfile'] = args.outfile

    if args.type=='fk':
    	result = module.SendCommand("RunForwardKinematics " + str(params))
    elif args.type=='jacobian':
    	result = module.SendCommand("RunJacobian " + str(params))
    else:
        print 'Unknown type options. Valid choices are: fk or jacobian.'
        

        
            
