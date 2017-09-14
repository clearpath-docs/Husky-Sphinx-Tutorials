Husky Gmapping Demo
======================================   

This tutorial shows you how to use `move_base <http://wiki.ros.org/move_base>`_ with `gmapping <http://wiki.ros.org/gmapping>`_ to perform autonomous planning and movement with simultaneous localization and mapping (SLAM), on a simulated Husky, or a factory-standard Husky with a laser scanner publishing on the scan topic.

To adapt this demo to your own Husky, you may need to clone the `husky_navigation <http://wiki.ros.org/husky_navigation>`_ repository, and modify the relevant parameters. To learn about `move_base <http://wiki.ros.org/move_base>`_, `gmapping <http://wiki.ros.org/gmapping>`_, and the `navigation <http://wiki.ros.org/navigation>`_ stack, see the `Navigation Tutorials <http://wiki.ros.org/navigation/Tutorials>`_.

Instructions
------------------

1.  Please make sure that the Husky navigation demo package is installed:

.. code:: bash

	$ sudo apt-get install ros-indigo-husky-navigation

2.  In three separate terminal windows:

	i.  Start the Clearpath-configured Husky simulation environment:

	.. code:: bash

		$ roslaunch husky_gazebo husky_playpen.launch

	ii. Start the Clearpath-configured rviz visualizer:

	.. code:: bash

		$ roslaunch husky_viz view_robot.launch

	iii.  Start the gmapping demo:

	.. code:: bash

		$ roslaunch husky_navigation gmapping_demo.launch

3.  In the Rviz visualizer, make sure the visualizers in the Navigation group are enabled.

4.  Use the 2D Nav Goal tool in the top toolbar to select a movement goal in the visualizer. Make sure to select an unoccupied (dark grey) or unexplored (light grey) location.

5.  As the robot moves, you should see the grey static map (map topic) grow. Occasionally, the gmapping algorithm will relocalize the robot, causing a discrete jump in the map->odom transform.

6.  To save the generated map, you can run the map_saver utility:

.. code:: bash

	$ rosrun map_server map_saver -f <filename>