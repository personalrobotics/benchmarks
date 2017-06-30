import numpy
import openravepy
from prpy.planning.snap import SnapPlanner
from prpy.planning.vectorfield import VectorFieldPlanner
from prpy.planning.logged import LoggedPlanner
from prpy.planning.cbirrt import CBiRRTPlanner

active_dof_indices = range(7)

# Feasible start/goal pair.
config_feasible_start = numpy.array([
  +2.35061574,  0.61043555,  0.85000000,  1.80684444, -0.08639935,
  -0.69750474,  1.31656172
])

config_feasible_goal = numpy.array([
  -0.84085883,  1.44573701,  0.20000000,  1.72620231, -0.81124757,
  -1.39363597,  1.29233111
])

openravepy.RaveInitialize(True)
openravepy.misc.InitOpenRAVELogging()
openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Fatal)

env = openravepy.Environment()

with env:
  env.Load('data/wamtest2.env.xml')
  robot = env.GetRobot('BarrettWAM')
  manipulator = robot.GetManipulator('arm')
  env.Remove(env.GetKinBody('floor'))

  robot.SetActiveManipulator(manipulator)
  robot.SetActiveDOFs(active_dof_indices)

# Create logged planner around snap planner
planner = LoggedPlanner(CBiRRTPlanner())

# Run planning query
robot.SetActiveDOFValues(config_feasible_start)

planner.PlanToConfiguration(robot, config_feasible_goal)