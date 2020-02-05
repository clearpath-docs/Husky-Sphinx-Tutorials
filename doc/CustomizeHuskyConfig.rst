Customize Husky Configuration
======================================

.. Note:: These tutorials assume that you are familiar with ROS and the catkin build system. Please familiarize yourself using the `ROS <http://wiki.ros.org/ROS/Tutorials>`_ and `catkin <http://wiki.ros.org/catkin/Tutorials>`_ tutorials.

If upgrading from a prior ROS release, you should now re-examine your backed-up files from Backing Up Husky Configuration to determine if there's any customizations that need to be configured on your platform.

Environment Variables
--------------------------

Husky's standard peripherals can be configured using these environment variables, to be added to the robot-wide setup file (/etc/ros/setup.bash). These environment variables are loaded on boot.

.. raw:: html

	<table class="wy-table-responsive"><tbody><tr><td><p><strong>Variable</strong> </p></td>
 	 <td><p> <strong>Default</strong> </p></td>
 	 <td><p> <strong>Description</strong> </p></td>
	</tr>
	<tr>  <td><span  id="line-37"></span><p> ROBOT_NETWORK</tt> </p></td>
 	 <td><p> None </p></td>
	  <td><p>  Configure a network interface to trigger the husky-core</tt> job, and initialize ROS_IP</tt>. If not set, husky-core</tt> will define ROS_HOSTNAME</tt> instead (see <a href="/ROS/NetworkSetup#Name_resolution">Network Setup</a>) </p></td>
	</tr>
	<tr>  <td><span  id="line-38"></span><p> HUSKY_IMU_PORT</tt> </p></td>
 	 <td><p> /dev/clearpath/imu</tt> </p></td>
 	 <td><p>  Port for the Husky UM6 IMU if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-39"></span><p> HUSKY_IMU_XYZ</tt> </p></td>
 	 <td><p> 0.19&nbsp;0.0&nbsp;0.149</tt> </p></td>
 	 <td><p>  Pose offset for the Husky IMU's standard mounting location </p></td>
	</tr>
	<tr>  <td><span  id="line-40"></span><p> HUSKY_IMU_RPY</tt> </p></td>
  <td><p> 0.0&nbsp;-1.5708&nbsp;3.1416</tt> </p></td>
  <td><p>  Orientation offset for the Husky IMU's standard mounting location </p></td>
	</tr>
	<tr>  <td><span  id="line-41"></span><p> HUSKY_NAVSAT_PORT</tt> </p></td>
	  <td><p> /dev/clearpath/gps</tt> </p></td>
	  <td><p> Port for the Husky GPS if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-42"></span><p> HUSKY_NAVSAT_BAUD</tt> </p></td>
	  <td><p> 19200</tt> </p></td>
	  <td><p> Baudrate for the Husky GPS </p></td>
	</tr>
	<tr>  <td><span  id="line-43"></span><p> HUSKY_UR5_IP</tt> </p></td>
	  <td><p> None </p></td>
	  <td><p> IP Address for the UR5 manipulator if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-44"></span><p> HUSKY_UR5_ENABLED</tt> </p></td>
	  <td><p> false</tt> </p></td>
	  <td><p> Enable/disable the UR5 manipulator. </p></td>
	</tr>
	<tr>  <td><span  id="line-45"></span><p> HUSKY_LMS1XX_IP</tt> </p></td>
	  <td><p> None </p></td>
	  <td><p> IP Address for the SICK LMS1XX LIDAR if present, must be set before running husky_bringup&nbsp;install</tt> </p></td>
	</tr>
	<tr>  <td><span  id="line-46"></span><p> HUSKY_LMS1XX_ENABLED</tt> </p></td>
	  <td><p> false</tt> </p></td>
	  <td><p> Enable/disable the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-47"></span><p> HUSKY_LMS1XX_XYZ</tt> </p></td>
	  <td><p> 0.2206&nbsp;0.0&nbsp;0.00635</tt> </p></td>
	  <td><p>  Pose offset for the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-48"></span><p> HUSKY_LMS1XX_RPY</tt> </p></td>
	  <td><p> 0.0&nbsp;0.0&nbsp;0.0</tt> </p></td>
	  <td><p>  Orientation offset for the SICK LMS1XX LIDAR </p></td>
	</tr>
	<tr>  <td><span  id="line-49"></span><p> HUSKY_TOP_PLATE_ENABLED</tt> </p></td>
	  <td><p> true</tt> </p></td>
	  <td><p> Enable/disable the standard Husky top plate. </p></td>
	</tr>
	</tbody></table>

Adding a Source Workspace
---------------------------

Configuring non-standard peripherals requires a source workspace on the robot PC.

1.  Create a new workspace:

.. code:: bash

	 $ mkdir -p ~/husky_kinetic_ws/src

2.  Add any custom source packages to the ~/husky_kinetic_ws/src directory.

3.  After adding your packages, make sure any necessary dependencies are installed:

.. code:: bash

 	$ cd ~/husky_kinetic_ws/
	$ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

4.  Build the workspace:

.. code:: bash

 	$ cd ~/husky_kinetic_ws/
	$ catkin_make

5.  Modify your robot-wide setup file (/etc/ros/setup.bash) to source your new workspace instead of the base kinetic 		install:

.. code:: bash

 	source /home/administrator/husky_kinetic_ws/devel/setup.bash

6.  Reinitialize your environment so that it picks up your new workspace:

.. code:: bash

 	$ source /etc/ros/setup.bash


Robot Description
-----------------------

In ROS Hydro and earlier, custom Husky descriptions (URDFs) were provided to customers in a workspace in their home folder. Since the Husky URDF has undergone some changes for Kinetic, your robot description from prior ROS releases will have to be slightly adapted.

First create a new URDF file in which you will define your custom Husky additions.  e.g. ``/home/administrator/husky-custom.xacro``.  Then modify ``/etc/ros/setup.bash`` to define the HUSKY_URDF_EXTRAS variable to point to your new file:

.. code:: bash

	export HUSKY_URDF_EXTRAS=/path/to/your/custom-file.xacro

So for the previous example, if we saved the customized file to ``/home/administrator/husky-custom.xacro`` we would put ``export HUSKY_URDF_EXTRAS=/home/administrator/husky-custom.xacro`` in the ``setup.bash`` file.

Modify your customized ``*.xacro`` file to add whatever additional features are required.  When finished, restart ROS by running `sudo systemctl restart ros`.  You can verify that your customized model is being used by running

.. code-block:: bash

		$ roslauch husky_viz view_robot.launch


Network Configuration
-----------------------

If upgrading from prior ROS releases, your old ``/etc/network/interfaces`` file may contain a static IP binding for your robot, or other customizations that should be replicated on your new setup.
