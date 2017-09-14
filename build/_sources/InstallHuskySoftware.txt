Install and Configure Husky Software
=======================================  

Installing Husky Software
---------------------------

Clearpath provides a lightly customized installation image of Ubuntu Trusty Server 14.04, that automatically pulls in all necessary dependencies for Husky software.


1.  Download the appropriate `Indigo Husky ISO image <http://packages.clearpathrobotics.com/stable/images/latest/indigo-husky/>`_ for your platform (32 bit - i386, 64 bit - amd64).

2. Copy the image to a USB drive using unetbootin:

.. code:: bash 

	 $ sudo unetbootin isofile="indigo-husky-amd64-latest.iso"

3.  Connect your robot PC to wired internet access, a keyboard, and a monitor. Make sure that the PC is connected to shore power, or the Husky battery is either fully charged.

.. warning:: The next step wipe your robot's hard drive, so make sure you have that image `backed up <http://wiki.ros.org/husky_bringup/Tutorials/Backing%20Up%20Husky%20Configuration>`_ on another system!

4.  Boot your robot PC from the USB drive, and let installer work it's magic.
5.  The setup process will be automated, and may take a long time depending on the speed of your internet connection.
6.  Once the setup process is complete, the PC will turn off. Please unplug the USB drive and turn the PC back on.
7.  On first boot, the username will be administrator and the password will be clearpath.
8.  Please follow the configuration instructions on the screen. If the computer reboots, wait for the PC to boot to the login screen, and re-enter the login credentials.
9.  Once the computer configuration is complete, you may use passwd utility to change administrator account password.
10. To setup a factory-standard Husky robot, ensure all your peripherals are plugged in, and run the following command:

.. code:: bash

	 $ rosrun husky_bringup install

The install script will configure a husky-core upstart service, that will bring up the base Husky launchfiles on boot. The script will also detect any standard peripherals (IMU, GPS, etc.) you have installed, and add them the service.

Testing base configuration
----------------------------

1.  To test your configuration, start the background service with the following command:

.. code:: bash

	 $ sudo service husky-core start

2.  The COMM light on your Husky should go from red to green. You can check that the service has started correctly by checking the logs:
 
.. code:: bash

	 $ sudo tail /var/log/upstart/husky-core.log -n 30

3.  Your husky should now be accepting commands from your joystick. The service will automatically start each time you boot your Husky's PC.


Calibrating the Magnetometer
---------------------------------

.. warning:: The Husky will rotate autonomously during calibration. Make sure all external cables are unplugged, and the Husky has unobstructed room to move in a 1 metre radius.

If your Husky has a UM6 IMU installed, you must calibrate the magnetometer for magnetic deviation before it will be used for pose estimation.

1.  Make sure the husky-core service is running.
2.  Execute the calibration script on the Husky computer remotely via ssh:

.. code:: bash

	rosrun husky_bringup calibrate_compass

3.  Follow the onscreen instructions.
