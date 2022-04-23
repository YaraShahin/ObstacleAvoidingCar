# Obstacle Avoiding Car-like Robot
## Introduction
`ObstacleAvoidingCar.py` is a robotics project made by level 4 electrical and electronics engineering students, Yara Shahin and Mohamed Abbas, as part of our computer engineering module. In this project, we attempt to use the python version of the robotics toolbox, developed by Peter Corke, to program a car-like robot to autonomously navigate to a user-input target while avoiding a number of obstacles placed randomly throughout the 20x20 map. The work on this project consisted of three main milestones: Algorithm planning, implementation, and evaluation. 
## Algorithm Planning
In this stage, both students brainstormed an algorithm on developing a theoretical approach to the obstacle avoidance task. We came up with our new algorithm after being inspired by the previous approaches to this task.
### Literature Review: Existing Obstacle Avoidance Algorithms
#### Fuzzy Logic Controller Technique
Technique used to divide a large problem into a set of sub-problems that are less complex. In the obstacle avoidance context, it works by dividing the surrounding area into ranges by angle and using “If-then” statements to determine which range would be best to move the robot into [1]. <img align="right" width="150" height="100" src="/images/figure1.png">
#### Bug 1 Algorithm
The robot moves towards the goal until it encounters an obstacle. The robot then takes a full revolution around the robot and determines the take-off point of the shortest path [2]. Beside this technique’s lack of effeciency, it would render ineffective in an environment of high obstacle density. 
#### Bug 2 Algorithm
The robot moves towards the goal through the calculated angle from the initial point until it encounters an obstacle. The robot moves along the sides of the obstacle until it finds a point with the same angle [3]. This algorithm is easier to implement and more efficient than bug 1 algorithm. Our algorithm is partially inspired by this algorithm as we will see in the later section.
#### Tangent Bug Algorithm
The robot finds tangents to the obstacle and calculates distances of robot from points where they touch the obstacle. The robot then takes path of the tangent which maximally decreases the distance which has to be crossed from the current position to the obstacle’s tangent and then to the goal [4]. This algorithm is very similar to the one we use in this project. The main difference is in the decision making as the robot chooses not the least distance path, rather the path of the least angle diversion from the goal’s. 
### Our Proposed Algorithm
- The robot calculates the angle to the goal and keeps moving in that direction with a fixed speed until it faces an obstacle. An obstacle is detected if it is within 5 units of the robot within the minimum range of angle the robot needs to move ahead in the goal’s direction. This range of angles is calculated based on the arc sin of the angle formed by the triangle in which the distance to the obstacle is the hypotenous and half the size of the robot’s front is a side, as shown in figure 2. <br/><img align="center" width="250" height="250" src="/images/figure2.jpg">
- Once the robot faces an obstacle, it loops over all the obstacles within 10 units. Assuming the obstacles are of points with negligable size, the robot calculates the angle on their lefts and rights that would fit the robot, as illustrated in figure 3. If an angle fits the robot according to the calculation illustrated above, the angle is appended in a list. <br/><img align="center" width="200" height="200" src="/images/figure3.jpg">
- The robot moves in the angle which would result in minimal diversion from the reference angle to goal. In extreme cases where the robot can’t find any fitting paths, the robot stops. 
#### This flowchart illustrates the algorithm: <img align="center" src="/images/ObstacleAvoidanceFlowChart.png">
## Implementation
### Software dependencies
- Python >3.6
- Roboticstoolbox python
- Matplotlib
- Math python library
#### In order to implement this project, we also used:
- Ubuntu 18.04
- Visual Studio code IDE
- Oracle vm virtualbox
- Ubuntu Terminal
- GitHub for version control
### Expected I/Os
#### Expected user Inputs
- `xf`: Goal x coordinate should be in the grid boundaries (between -20  and 20)
- `yf`: goal y coordinate should be in the grid boundaries (between -20  and 20)
- `x0`: initial point x coordinate should be in the grid boundaries (between -20  and 20
- `y0`: initial point y coordinate should be in the grid boundaries (between -20  and 20)
- `n_obstacles`: number of obstacles in the 20x20 areana (tested for <100 with >0.8 robot size)
- `robot_size`: assuming that the robot is a square, enter its side length (the icon will automatically scale)
- `img_path`: the absolute path of the robot icon png image. An image `redcar.png` is uploaded in the images folder that could be used. <img align="center" src="/images/redcar.png">
#### Expected outputs
- `"Value Error. The obstacle is too close." `: when there’s no way between the obstacles.
- `"So sorry, but I'm outta here."`: when the only solution between the obstacles is out the arena grid.
- `"So sorry, there's an unexpected error somehow? Can you pass me the laptop so I can double-check?"`: for unexpected errors
- Car plot: There should be a pop-up plot visualizing the car heading in the planned direction, as well as black markers for the random obstacles and a red marker for the set goal.
### Code Functionalities
#### move()
- `move(angle)`: function takes the angle and moves the robot in that angle with a constant speed of 1
- `besides(distance, angle)`: given the distance and angle of an obstacle to a robot, return the closest angles on the obstacle's left and right that will still fit the robot.
- `does_fit(angle)`: function does it fit? takes the angle we want to go in as input at returns 0 if the robot fits to go in this angle or the distance of the obstacle that wouldn't make it fit.
- `angle_picker(distance_to_goal, angle_to_goal)`: takes the calculated distance and angle to goal and outputs the obstacle-free angle in which the robot should go.
## Evaluation
### Results
The code has been tested with different initial and target positions, with number of obstacles up to 200. It has proven robust for up to 120 obstacles in the 20x20 grid areana. Please refer to `/images/testrun.mp4` for a sample testrun video <img align="center" src="/images/testrun.png">
## References 
[1] Talabattula Sai Abhishek et al 2021 IOP Conf. Ser.: Mater. Sci. Eng. 1012 012052 <br/>
[2] Lumelsky, V., Skewis, T., “Incorporating Range Sensing in the Robot Navigation Function.” IEEE Transactions on Systems, Man, and Cybernetics, 20:1990, pp. 1058–1068.  <br />
[3] Lumelsky, V., Stepanov, A., “Path-Planning Strategies for a Point Mobile Automaton Moving Amidst Unknown Obstacles of Arbitrary Shape,” in Autonomous Robot Vehicles. New York, Spinger-Verlag, 1990 <br />
[4] Bhavesh, V. A. (n.d.). Comparison of Various Obstacle Avoidance Algorithms. www.ijert.org <br />


