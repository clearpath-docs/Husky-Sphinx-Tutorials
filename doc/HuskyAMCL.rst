Husky AMCL Demo
====================================

This tutorial shows you how to use `move_base <http://wiki.ros.org/move_base>`_ with `amcl <http://wiki.ros.org/amcl>`_ to perform autonomous planning and movement with localization on a simulated Husky, or a factory-standard Husky with a laser scanner publishing on the scan topic.

To adapt this demo to your own Husky, you may need to clone the `husky_navigation <http://wiki.ros.org/husky_navigation>`_ repository, and modify the relevant parameters. To learn about `move_base <http://wiki.ros.org/move_base>`_, `amcl <http://wiki.ros.org/amcl>`_, and the navigation stack, see the `Navigation Tutorials <http://wiki.ros.org/navigation/Tutorials>`_.

Instructions
------------------

1.  Please make sure that the Husky navigation demo package is installed:

.. code:: bash

	$ sudo apt-get install ros-kinetic-husky-navigation

2.  In three separate terminal windows:

	i.  Start the Clearpath-configured Husky simulation environment:

	.. code:: bash

		$ roslaunch husky_gazebo husky_playpen.launch

	ii. Start the Clearpath-configured rviz visualizer:

	.. code:: bash

		$ roslaunch husky_viz view_robot.launch

	iii.  Start the amcl demo:

	.. code:: bash

		$ roslaunch husky_navigation amcl_demo.launch

3.  In the Rviz visualizer, make sure the visualizers in the Navigation group are enabled.

4.  Use the 2D Pose Estimate tool in the top toolbar to give amcl an initial pose estimate. Without an initial estimate, the Monte Carlo localization approach is unlikely to converge the correct pose.

5.  Use the 2D Nav Goal tool in the top toolbar to select a movement goal in the visualizer. Make sure to select an unoccupied (dark grey) or unexplored (light grey) location.

6.  Note that in this example, the robot uses data from the laser scanner to correct the fused odometry estimate, and mitigate drift. The amcl node uses a pregenerated map of this environment to compare against incoming scans. See the next tutorial for a demo with localization and mapping.
