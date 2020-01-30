Simulating Husky
==================

Husky can be used in both real and simulated environments. If you are using a physical Husky, please make sure all your peripherals are plugged in, turn on the robot, and skip directly to step 2.

Husky can be simulated in several environments using the husky_gazebo packages. To start a simulation:

1.  Make sure the simulation package is installed:

.. code:: bash

	$ sudo apt-get install ros-kinetic-husky-simulator

2. Run one of the two provided simulation environments:

i.  Simulate Husky in an empty world. You can add new objects to this world using the Gazebo controls (Gazebo Tutorial - Building a World.

.. code:: bash

	$ roslaunch husky_gazebo husky_empty_world.launch

ii.  Simulate Husky in a Clearpath designed world. This is the base environment for the navigation tutorials. It will take some time to start, as the simulator will need to download resources from the Gazebo servers.

.. code:: bash

	$ roslaunch husky_gazebo husky_playpen.launch

3.  Pick your own world in which to simulate Husky (Using roslaunch with Gazebo).

.. code:: bash

	$ roslaunch husky_gazebo husky_playpen.launch
