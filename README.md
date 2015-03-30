# OpenRAVE benchmark tools
Benchmarks to measure OpenRAVE's performance.

# Current Results
The following are the latest results.

<img src="https://github.com/personalrobotics/or_benchmarks/blob/master/results/self_collision.cps.png" alt="self_collision_results" width="250"/>
<img src="https://github.com/personalrobotics/or_benchmarks/blob/master/results/empty_env_collision.cps.png" alt="empty_env_collision_results" width="250"/>
<img src="https://github.com/personalrobotics/or_benchmarks/blob/master/results/prkitchen_collision.cps.png" alt="prkitchen_env_results" width="250" />

# Installation
The following packages must be installed:
* ros-hydro-moveit-core
* ros-hydro-moveit-ros-planning-interface
* libgoogle-perftools-dev
```
cmd> sudo apt-get install ros-hydro-moveit-core ros-hydro-moveit-ros-planning-interface \
libgoogle-perftools-dev
```

The following .rosinstall can be used to download the important packages into a catkin workspace:

```
- git: {local-name: or_benchmarks, uri: 'git@github.com:personalrobotics/or_benchmarks'}
- git: {local-name: herbpy, uri: 'git@github.com:personalrobotics/herbpy'}
- git: {local-name: prpy, uri: 'git@github.com:personalrobotics/prpy'}
- git: {local-name: herb_description, uri: 'git@github.com:personalrobotics/herb_description'}
- git: {local-name: or_urdf, uri: 'git@github.com:personalrobotics/or_urdf'}
- git: {local-name: openrave_catkin, uri: 'git@github.com:personalrobotics/openrave_catkin'}
- git: {local-name: ss_plotting, uri: 'git@github.com:personalrobotics/ss_plotting'}
- git: {local-name: or_fcl, uri: 'git@github.com:personalrobotics/or_fcl'}
- git: {local-name: pr-ordata, uri: 'git@github.com:personalrobotics/pr-ordata'}
```

# Kinematic Benchmarks
The kinematick benchmark script can be used to profile forward kinematic computation and Jacobian computation.  

To profile forward kinematics:
```
cmd> rosrun or_benchmarks run_kinematics_benchmark.py --type fk
```
By default this will randomly sample 50,000 configurations for the right arm of HERB and compute forward kinematics for each of these configurations.  To test a different manipulator, use the ```--manip``` flag:
```
cmd> rosrun or_benchmarks run_kinematics_benchmark.py --type fk --manip head
```

To profile Jacobian computation:
```
cmd> rosrun or_benchmarks run_kinematics_benchmark.py --type jacobian
```

Use the ```--help``` flag to see all options for the script:
```
cmd> rosrun or_benchmarks run_kinematics_benchmark.py --help
```

# Collision Benchmarks
The collision benchmark script can be used to profile both self collision checking and environment collision checking.

To run all benchmarks and generated updated statistics simply run:
```
cmd> rosrun or_benchmarks run_all.py
```
This will run three sets of tests:
* Self collision - 50000 poses read from ```datasets/self_benchmark.test```
* Collision against an empty environment - 50000 poses read from ```datasets/env_benchmark.test```
* Collision against the pr_kitchen environment - 50000 poses read from ```datasets/env_benchmark.test```
Each test will be run using the ode, pqp and fcl collision checkers.

The script will generate a new set of ```.png``` files which compare timing information across the three collision checking engines.  These new files will be written to the ```results``` directory.  To save these results, and update this webpage, just commit the new files.

The ```run_all.py``` script utilizes the ```run_collision_benchmark.py``` script. Use ```run_collision_benchmark.py``` directly to run individual tests.  The ```--help``` flag will show all options for the script:
```
cmd> rosrun or_benchmarks run_collision_benchmark.py --help
```
