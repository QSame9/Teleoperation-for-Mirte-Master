# Teleoperation-for-Mirte-Master

![Alt text](/Images%20videos%20and%20slides/Teleoperations_interface_as_seen_from_front.png "Teleoperation interface as seen from front, image by Duncan Bijkerk and Quita Carriere")

Authors: Ynze Visser, Quita Carriere, Aditya Swami, Duncan Bijkerk  
Repository: [GitHub page](https://github.com/QSame9/Teleoperation-for-Mirte-Master)  

A physical remote controller interface for the Mirte Master. This project is part of the Bachelor End Project for Mechanical Engineering students at the TU Delft. It was commissioned by the Cognitive Robotics department of the TU Delft.  

# Use of the Teleoperation interface
The Teleoperation interface, here on out also called remote controller, is made for the Mirte Master,  
 often called Mirte, for more info see the [Mirte site](https://mirte.org/) and the [Mirte GitHub](https://github.com/mirte-robot).

![Alt text](/Images%20videos%20and%20slides/mirte_master.jpeg "Mirte Master, image by Chris Pek")

The arm movement of the Mirte is controlled by the arm of the controller, mimicking the angular position of each joint. All joints can therefor be controlled in 0th order. To reach the full workspace of the Mirte, a 1st order movement is coded into the periferal working space of the yaw joint of the controller. The Mirte yaw joint can only move 180&deg;, so to reach the back side of the Mirte one of the swithes is made to mirror the movement of all of the arm joints except for the yaw.

The gripper of the Mirte is controlled by the pincher mechanism of the controller, see image below.

![Alt text](/Images%20videos%20and%20slides/Stripped%20pincher%20model.png "Pincher CAD close up")

Here the linear pinching is transfered via a rack and pinion mechanism to be read out by an angular encoder. This is mapped to the workspace of the gripper. The information is also sent to a force simulator, as to get the force that the force feedback needs to apply. This force feedback is done through Series Elestic Actuation. The servo motor is turned to make the tortion spring compress the correct amount to apply a stiffness to the pincher tabs that the operator holds. The other spring in the mechanism is a return spring, so the tabs open again once the operator lets go.

The driving is done through a simple, off-the-shelf joystick. The Mirte can drive in both holonomic and angular drive methods, and a switch allows for changing the driving style. Due to sensor bias in the joystick, a calibration is done during startup, so the neutral position is determined.

[Here](https://youtube.com/shorts/fPgHeI3LzEk) is a video of the teleoperation interface in action. Some of the delay issues have been resoved since the making of this video.


All the files of the software, CAD models, and electronics can be found in this GitHub repository. The written report for the Bachelor End Project, as well as the presentations given for the course and the files used for analysis can be found in this repository as well. 

