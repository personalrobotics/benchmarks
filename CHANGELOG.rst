^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_benchmarks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2015-05-07)
------------------
* Updating results with latest versions of code. Changing dataset to contiain 10000 points instead of 20000.
* Making moveit an optional depedency. Updating analyze to work with latest ss_plotting.
* Updating benchmarks
* Increasing font size
* Adding functionality to save off all plots as png
* Adding script to run all benchmarks and automagically generate plots for wiki. Adding environment locking. Adding test files for environment and self collision testing.
* Added helper program for using gperftools.
* Added a helper script for testing memory leaks.
* MoveIt! FK and Jacobian benchmarks.
* Updating benchmark code to time the two individual pieces of FK seperately.  Adding helper function for computing milliseconds elapsed to Datautils.
* Removing dead code. Cleaning up documentation. Adding doxygen configuration file.
* Reorganizing and adding forward kinematics and jacobian benchmarks.
* Modified the MoveIt benchmark to accept poses YAML
* Added the MoveIt profile.
* Fixed some build issues in Catkin.
* Catkin-ized.
* Added ability to reload a saved file
* Updating to include multiple collision checkers and the ability to perform self collision tests
* Initial collision checking benchmark code
* Contributors: Jennifer King, Michael Koval, jeking
