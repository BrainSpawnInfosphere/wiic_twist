# wiic_twist 

**Original Author(s):** Gabriele Randelli, modified by Kevin J. Walchko

**License:** GPL v3

**Original Repository:** https://wiic.svn.sourceforge.net/svnroot/wiic

**Website:** http://wiic.sourceforge.net

**Dependencies:** None

**Description:** Utilizes the wiic library ported to [http://www.ros.org ROS] and example 
code that talks to the Wii controller to publish twist messages to a robot. The wiic 
library can detect multiple wiimotes and talk to them.

	git clone git://github.com/walchko/wiic_twist.git
	rosmake wiic_twist
 	rosrun wiic_twist wiic_twist
  	press buttons 1 + 2 for the wiimote to sync
  	when done, either use the power button on the wiimote or ^C to exit

## Works
* wiimote - all buttons, accel, gyro, lights, vibration 

## Doesn't Work
* nunchuck
* motion plus - program hangs when you try to use it.
* balance board (You can connect, but it doesn't continuously read it)

## Unknown
* Guitar Hero

## Changes

**Sep 2012** Wiic is now a homebrew rosdep requirement. Current wiic has an issue with
Mountain Lion and doesn't connect to wiimote ... working issue.

**Jan 2011** Modified example code and ported to ROS. Compiles and works under OS X 
10.6.6 and ROS Diamondback