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
	
	 $ mkdir -p ~/husky_indigo_ws/src

2.  Add any custom source packages to the ~/husky_indigo_ws/src directory.

3.  After adding your packages, make sure any necessary dependencies are installed:

.. code:: bash

 	$ cd ~/husky_indigo_ws/
	$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

4.  Build the workspace:

.. code:: bash

 	$ cd ~/husky_indigo_ws/
	$ catkin_make

5.  Modify your robot-wide setup file (/etc/ros/setup.bash) to source your new workspace instead of the base indigo 		install:

.. code:: bash
 	
 	source /home/administrator/husky_indigo_ws/devel/setup.bash

6.  Reinitialize your environment so that it picks up your new workspace:

.. code:: bash

 	$ source /etc/ros/setup.bash

7.  Augment the husky-core job with launch files from your custom packages:

.. code:: bash

	$ rosrun robot_upstart install my_custom_package/launch --job husky-core --augment


Robot Description
-----------------------

In ROS Hydro and earlier, custom Husky descriptions (URDFs) were provided to customers in a workspace in their home folder. Since the Husky URDF has undergone some changes for Indigo, your robot description from prior ROS releases will have to be slightly adapted.

To create a custom Husky configuration, fork the `husky_customization <https://github.com/husky/husky_customization.git>`_ repository to your `GitHub account <http://wiki.ros.org/GitHub>`_, and clone the fork into your workspace:

.. code:: bash

	$ cd ~/husky_indigo_ws/src
	$ git clone https://github.com/<username>/husky_customization.git -b indigo-devel
	$ cd ~/husky_indigo_ws
	$ catkin_make
	$ source devel/setup.bash

To modify your Husky's URDF description (see `ROS URDF Tutorials <http://wiki.ros.org/urdf/Tutorials>`_), edit the ``husky_customization/husky_custom_description/urdf/custom_description.urdf.xacro`` file. Run roslaunch husky_viz view_model.launch to see your custom model in rviz.

To modify your Husky's simulation configuration (see `Gazebo's URDF Tutorials <http://gazebosim.org/tutorials/?tut=ros_urdf>`_), edit the ``husky_customization/husky_custom_gazebo/urdf/custom_description.gazebo.xacro`` file. Run ``roslaunch husky_gazebo husky_playpen.launch`` to see your custom Gazebo configuration in action!

Once you are done customizing your Husky configuration, don't forget to commit and push the changes back into your `GitHub <http://wiki.ros.org/GitHub>`_ repository.

Network Configuration
-----------------------

If upgrading from prior ROS releases, your old ``/etc/network/interfaces`` file may contain a static IP binding for your robot, or other customizations that should be replicated on your new setup.



