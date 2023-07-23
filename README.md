# ROS2-PotentialField
ROS2 Package for turtlebot3 to execute local path planning using the potential field methods.
This package is an excuse to learn C++ and ROS2. It's not complete nor plug and play for now but feel free to use it as you want :D

![alt text](https://github.com/Tim-HW/ROS2-PotentialField/blob/main/potentialfiled.jpg)

## Potential Field paradigm
The potential field method is a widely used technique in robotics and motion planning to guide a robot towards a desired goal while avoiding obstacles in its environment. The method operates based on the concept of potential fields, where each point in the robot's environment is assigned a "potential" value.

Here's a step-by-step explanation of the potential field method:

    Goal Attraction: The robot is assigned a "goal" location, which represents the desired position it needs to reach. The goal location is associated with an attractive potential that pulls the robot towards it. The potential field around the goal creates a force that influences the robot's motion, directing it towards the goal.

    Obstacle Repulsion: Obstacles in the environment are assigned repulsive potentials. These potentials create a repulsive force that pushes the robot away from obstacles. The strength of the repulsion increases as the robot gets closer to the obstacles, preventing collisions.

    Superposition: The total potential at any given point in the environment is the combination of the attractive potential towards the goal and the repulsive potential due to nearby obstacles. The robot calculates the net force at its current position based on these potential fields.

    Motion Planning: Using the net force calculation, the robot determines the direction it should move in to minimize the potential function. Typically, the robot follows a gradient descent approach, which means it moves in the direction opposite to the gradient of the potential function at its current position. This allows the robot to naturally move towards the goal and avoid obstacles.

    Iterative Process: The robot repeats this process multiple times, making small adjustments to its position based on the current net force calculation. This iterative process continues until the robot reaches the goal or a predefined termination condition is met.

Advantages of the Potential Field Method:

    It is relatively simple to implement and computationally efficient, making it suitable for real-time applications.
    The method naturally handles dynamic environments, as obstacles are continuously avoided based on their current positions and the robot's movement.
    It allows for the incorporation of different weights and functions for attractive and repulsive potentials, enabling fine-tuning of the robot's behavior.

However, the potential field method also has some limitations:

    It may suffer from local minima, where the robot becomes trapped in a potential well and cannot reach the goal.
    In complex environments, the robot may experience oscillations or get stuck in narrow passages between obstacles.
    It does not guarantee optimality in finding the shortest path to the goal, as it relies on local information rather than global planning.

In practice, the potential field method can be combined with other planning algorithms to address its limitations and improve the robot's overall performance.