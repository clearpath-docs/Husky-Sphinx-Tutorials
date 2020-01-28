Husky Frontier Exploration Demo
======================================

This tutorial shows you how to use `move_base <http://wiki.ros.org/move_base>`_ with `gmapping <http://wiki.ros.org/gmapping>`_ and `frontier_exploration <http://wiki.ros.org/frontier_exploration>`_ to perform autonomous planning movement, and exploration with simultaneous localization and mapping (SLAM), on a simulated Husky, or a factory-standard Husky with a laser scanner publishing on the scan topic.

To adapt this demo to your own Husky, you may need to clone the `husky_navigation <http://wiki.ros.org/husky_navigation>`_ repository, and modify the relevant parameters. To learn about `move_base <http://wiki.ros.org/move_base>`_, `gmapping <http://wiki.ros.org/gmapping>`_, `frontier_exploration <http://wiki.ros.org/frontier_exploration>`_ and the `navigation stack <http://wiki.ros.org/navigation>`_, see the `Navigation Tutorials <http://wiki.ros.org/navigation/Tutorials>`_.


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

	iii.  Start the frontier_exploration demo:

	.. code:: bash

		$ roslaunch husky_navigation exploration_demo.launch

3.  In the Rviz visualizer, make sure the visualizers in the Navigation group are enabled.

4.  Use the Point tool in the top toolbar to draw a closed polygon on the map that the Husky should explore. Watch the terminal window for instructions.

5.  As the robot moves, you should see the grey static map (map topic) grow. Occasionally, the gmapping algorithm will relocalize the robot, causing a discrete jump in the map->odom transform.

6.  When the exploration goal is complete, you will see a feedback message in the terminal window. You can now issue a new exploration goal if you wish.

7.  To save the generated map, you can run the map_saver utility:

.. code:: bash

	$ rosrun map_server map_saver -f <filename>
