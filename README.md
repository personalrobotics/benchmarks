# OpenRAVE benchmark tools
Benchmarks to measure OpenRAVE's performance.

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

To profile self collision:
```
cmd> rosrun or_benchmarks run_collision_benchmark.py --self
```
This will generate 50000 poses for the HERB robot and check self collision in each pose.

Use the ```--help``` flag to see all options for the script:
```
cmd> rosrun or_benchmarks run_collision_benchmark.py --help
```
