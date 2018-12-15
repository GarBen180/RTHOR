# RTHOR
Robotic Third Hand for Object Retrieval

We were able to generate object location data using the Pixy2 camera, & develop the movement of the robotic arm. We were not able to complete the voice command interface for controlling the project. We initially started learning as much as we could about object tracking using Tensorflow and an Xbox Kinect. Due to the time constraints of the semester, we chose to use the Pixy2 camera because it already contains object tracking software. We were able to use this software to generate x and y coordinates of objects that we taught the Pixy2. These coordinates drive the motions of the robotic arm using inverse kinematics. The arm retrieves the object requested, brings it to the drop off point, and returns to the start position. In the future, adding vocal commands would make the system more effective, and using Tensorflow would expand the ability of the arm to recognize more objects. Also, the design of the robotic arm would best be implemented using a cartesian base for range of motion, and an articulating arm for object oriented alignment and retrieval. 

This project has been on my to-do list ever since I started pursuing a BS in Computer Engineering. I had the opportunity to start development to fulfill a project requisite during my final semester. This project was presented at the Research and Creative Works Conference at BYU-I Dec. 13, 2018.

Robotics and Artificial Intelligence play an increasingly important role in many aspects of modern life. From manufacturing and production to medicine and defense, these two topics go hand in hand to deliver optimal results and improve quality of life. The purpose of this project is to develop a robotic system that can be used in home and industry automation, nick-named RTHOR (pronounced Arthur). Topics explored include image recognition and processing, robotic motion and control, and voice command recognition. 

The design involves:
1.	Object tracking algorithms to teach the system what an object is
2.	Vocal recognition algorithms to receive vocal commands
3.	Kinematics algorithms to control robotic motion

Further Development:
1.	Home Automation using devices such as the Amazon Echo or Google Home
2.	Using something like Tensorflow to refine and expand capabilities
3.	Hardware redesign for maximum functioning area

Hardware:
•	Raspberry Pi 3 B+
•	MSP432P401R
•	Serial Servos
•	Servo Debug Board
•	Pixy2 Camera & Software
•	Aluminum Mount Hardware
•	Code Composer Studio
•	Camera Mount
•	Robotic Arm Base and Frame
