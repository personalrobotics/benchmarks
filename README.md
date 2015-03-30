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
```
