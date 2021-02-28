openni2_camera_param

This project is to change how the ROS2 OpenNI2 Driver ported by Mike Fergusson controls stream publishing by using parameters because the method of checking for subscribers doesn't currently work.

Considerable code has been refactored out of openni2_device to hopefully make it easier to understand and maintain.

Initially, the publishing method was changed to use standard publishers.  Now I've added a conditional compile flag to give a choice between using ROS2's Image Transport and standard publishers.
