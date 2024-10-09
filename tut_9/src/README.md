
/**

@mainpage SPRINT_3

Code Summary

ImageOverlay - tutorial_9_3_2 node:
This code uses OpenCV to overlay a .pgm image onto a .png background. It processes the .pgm image by filtering out non-black pixels and applies transformations like rotation and resizing to ensure proper alignment with the background. The images are then combined, displayed, and saved as a final result.

Cylinder - tutorial_9_3_6:
This code processes LIDAR laser scan data to detect cylindrical objects in the environment. It subscribes to laser scan and odometry topics, segments the scan data to identify potential cylinders, and then publishes visualization markers of the detected cylinders. It uses odometry data to maintain the robot's reference frame while detecting objects.




