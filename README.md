# Multi-agent Differential Robot Path Planning with Visual Servoing
<ins>Team Members</ins>  
Temple Shema -   
Kartik Doddikindi -   
Jinsol Kang -   
Moses Muamba -   

## Introduction
ECSE 275 introduced a variety of concepts covering a wide range of applications. Among these, what intrigued our group the most was the study of differential robots and their path planning. Mobile robots have numerous applications, particularly in autonomously planning their journeys to reach a destination. This motivated us to delve deeper and explore how we could expand on these ideas.  
As a result, we decided to implement multiple differential robots operating simultaneously. Additionally, we integrated visual servoing into the path-planning process, enabling the robots to perform more complex tasks, such as identifying secondary goal points that are unknown at the start of their journey.  
By combining these steps, we aimed to create a system of three differential robots capable of navigating toward their primary goals, discovering unknown secondary goal points using visual servoing, and avoiding collisions with obstacles and each other. Each robot would be given a color, red, green, and blue, so they can find their respective secondary goals that are red, green, or blue.

## Approach
***Differential Robot***

***Visual Servo***
To implement visual servoing, we equipped the mobile robot with a vision sensor capable of depth perception up to 3 meters. We determined that 3 meters was an acceptable range because it allowed the robot to detect secondary goals without requiring excessive processing time to calculate their positions. (add image)

To simulate the secondary goals assigned to the robots, we created colored spheres that the vision sensor could recognize. These spheres served as visual targets for the robots to identify and navigate toward. (add image)

---
*Version 1*  
We needed to develop a method for the robot to center itself on its target (ball) when it detected its respective colored ball. The initial version of the visual servo system used proportional control. Based on a predefined RGB threshold, the system scanned every pixel in the vision sensor's field of view and calculated the average position of the pixels that matched the color threshold. Using this average position, it determined the offset from the center of the vision sensor, providing an x-coordinate offset in pixels. With this value and a proportional gain factor (*kₚ*), we implemented proportional control to enable the robot to rotate toward the target. (Add gif)

<ins>Limitation<ins/>
- Multiple balls could not be tracked because the system averaged their positions, leading to inaccurate results (add gif)
- X-coordinate offset alone could not guid the robot to move towards the goal (add gif)
---
*Version 2*  
Upon further inspection, we discovered a more effective function within the vision sensor: `simVision.blobDetectionOnWorkImg`. This function groups pixels that form an image of a ball into "blobs," allowing manipulation of each blob individually. Additionally, the vision sensor includes a depth sensor, enabling the detection of the distance between the camera and the balls.

Using these capabilities, we programmed the mobile robot to identify the balls, select the closest one using the depth sensor, and calculate the normalized depth. This depth value was then used in a proportional control system to regulate the robot’s forward speed. As a result, the robot could identify the nearest secondary goal and move toward it. (add gif)

***Potential Field***

***Flow Chart***

## Results

## Conclusion
