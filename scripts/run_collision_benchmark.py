#!/usr/bin/env python
import argparse, herbpy, openravepy, os

# Thanks to Chris Dellin for this nice hack. :-)
if ord(os.environ.get('ROS_DISTRO', 'hydro')[0]) <= ord('f'):
    package_name='or_benchmarks'
    import roslib; roslib.load_manifest(package_name)
    import prpy; prpy.dependency_manager.export(package_name)

def run_benchmark(body, engine, testfile=None, self_collision=False, envfile=None, 
                  random=50000, extent=2.0, viewer=None,  outfile=None ):

    # Load the environment
    if body == 'herb':
        env, robot = herbpy.initialize(sim=True, attach_viewer=viewer)
    else:
        env = openravepy.Environment()

    if envfile:
        env.Load(envfile)
        
    # Set the collision checker
    cc = openravepy.RaveCreateCollisionChecker(env, engine)
    if(engine == 'sdf'):
        cc.SendCommand('SetSubChecker ode')
    cc.SetCollisionOptions(0)
    if cc is None:
        raise Exception('Invalid collision engine. Failing.')
    env.SetCollisionChecker(cc)

    # Verify the kinbody is in the environment
    body = env.GetKinBody(body)
    if body is None:
        raise Exception('No body with name %s in environment. Failing.'
                        % body)

    # Load the openrave module
    try:
        module = openravepy.RaveCreateModule(env, 'collisioncheckingbenchmark')
    except openravepy.openrave_exception:
        raise Exception('Unable to load CollisionCheckingBenchmark module.'
                        ' Check your OPENRAVE_PLUGINS environment variable.')

    # Generate the parameters
    params = {}
    params['body'] = str(body.GetName())
    params['random'] = random
    if outfile is not None:
        params['outfile'] = outfile
    params['extent'] = extent
    if self_collision:
        params['self'] = True
    if testfile:
        params['datafile'] = testfile

    with env:
        result = module.SendCommand("Run " + str(params))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run the benchmark tests")
    parser.add_argument("--outfile", type=str, default=None,
        help="The output file to save results in, if none results are not saved")
    parser.add_argument("--env", type=str, default=None,
        help="The environment to load")
    parser.add_argument("--body", type=str, default="herb",
        help="The kinbody to move around for collision checking")
    parser.add_argument("--test", type=str, default=None,
        help="The list of poses to check, this should be in the form of a"
             " results file from a previous run")
    parser.add_argument("--random", type=int, default=50000,
        help="The number of random poses to check. This will be ignored if the"
             " test parameters is set.")
    parser.add_argument("--extent", type=float, default=2.0,
        help="The edge length for the cube from which poses will be sampled.")
    parser.add_argument("--engine", type=str, default="ode",
        help="The underlying physics engine to use for collision checking")
    parser.add_argument("--self", action='store_true',
        help="If true, run self collision checks instead of environment"
             " collision checks")
    parser.add_argument("--viewer", type=str, default=None,
        help="The viewer to attach to the environment")
    args = parser.parse_args()
    
    run_benchmark(args.body, args.engine,
                  testfile = args.test,
                  self_collision = args.self,
                  envfile = args.env,
                  viewer = args.viewer,
                  random = args.random,
                  outfile = args.outfile,
                  extent = args.extent
                  )
