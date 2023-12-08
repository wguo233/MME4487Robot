# MME4487Robot
Lab 003 Group 2  

Wesley Guo, Mahmoud Abdelmageed, Saad Nezami  

Robots are emerging as a technology that can perform repetitive or dangerous tasks instead of humans. The objective of the robot is to collect objects of high commercial value while rejecting other objects. As a typical mechanical design project, the robot should be light and compact. The robot should depart from a base location, gather objects, and return them to base location in a semi-autonomous manner. The robot will be remotely controlled by an ESP-based microcontroller. A camera will be provided to provide visual feedback to the operator. The robot should be able to survive communication dropouts. The robot will be tested for operating range, accuracy, repeatability, reproducibility, sensitivity, resolution, and any other performance measures. The robot will be equipped with DC and servo motors and an LED to ensure successful communication between the robot and the controller. Further design improvements are recommended to ensure smoother drive and more reliable sorting mechanisms.  
An ESP32 microcontroller is used to control the robot and its motors. The robot is wirelessly connected to another ESP32 via ESPNOW which is used to control the robots actions from short to mid-range distances. The robot features full ranged movement on a flat surface with turning capabilities and speed control. It is designed to drive up to an object where the operator then presses a button on the controller to initiate the sorting process of the robot. The gripper used to latch onto objects will close and a colour sensor inside will return R,G,B, and C values to determine if an object is of value or not. Objects that are picked up are brought to a storage container on the robot by a servo controlled arm. These objects are placed on top of a servo controlled platform, which is controlled using another button on the controller. Upon pressing the drop-off button, the servo motor holding up the platform will rotate which will cause inside objects to slide out. After a delay, the servo returns back to its original position.  
The pickup and sorting system is comprised of an arm and a gripper, both controlled using an RC servo motor. The colour detector is placed inside the gripper and will read values when the gripper is closed. The system is initiated using a push button to prevent unintended actions when objects are approaching the colour sensor, but not in the desired place for pickup.  

The process:  
1. On startup, the gripper will close and the colour detector will read RGB values and sum them. This will be the value of ambient light. Once finished, the gripper will open back up. This happens once.  
2. When pushing the button, the gripper will close. Objects will be encapsulated if any are present.
3. When the gripper is in the closed position, the colour detector will read R,G,B,C values.
4. If the resulting sum of the RGB values is similar to the ambient light recorded, it assumes there is no object, and the gripper will open again.  
5. If there is a difference in the recorded summation of RGB values and the ambient light, it assumes there is an object.
6. If the R,G,B values fall within certain ranges, the robot will consider it as a desired object.
7. For the desired object, the arm will rotate backwards towards the chassis and drop off the object, then rotate back to starting position.
8. For the undesired object, the arm will rotate 90 degrees upwards, then return to starting position and let go of the object.
