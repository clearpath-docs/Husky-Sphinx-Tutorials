Backing Up Husky Configuration
=================================


Upgrading your Clearpath Husky to ROS Kinetic from older ROS distributions is a straightforward process, however it's important to understand that each Husky is different, having undergone customization to your specifications by our robotsmiths.

Please take the time to understand what these modifications are, and how to recreate them on your fresh install of Ubuntu Xenial/ROS Kinetic.

Performing a Backup
-----------------------------


1.  As a fail-safe, please make an image of your robot's hard drive. You should always be able to restore this image if you need to revert back to your previous configuration.

*  The easiest approach may be to either connect a removable (USB or similar) hard drive to the robot PC, or to unplug the robot hard drive and 	insert it into a PC or workstation.
*  You can then use a tool such as CloneZilla or dd to write a backup image of your robot's hard drive onto another hard drive.
*  Alternatively, you can simply replace the robot computer's hard-drive, reserving the Hydro-configured drive and installing a new one to use with Kinetic.

2.  There are several places in the filesystem you should specifically look for customizations for your Husky:

====================================	====================================================
Location:								Description:
====================================	====================================================
/etc/network/interfaces					Your robot may have a custom network configuration configured in this file.
/etc/ros/hydro/husky-core.d/*.launch	Will contain base.launch and description.launch, may contain custom launch files for your robot configuration.  Replaced by ros.d in newer versions.
/etc/ros/hydro/ros.d/*.launch					Will contain base.launch and description.launch, may contain custom launch files for your robot configuration.  Replaces husky-core.d in newer versions.
/etc/ros/setup.bash						May contain environment variables for your configuration.
====================================	====================================================

3.   Please save all these files and use them as a reference during Kinetic configuration!
