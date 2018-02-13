# Reinforcement learning approach to make a snake robot learn the optimal gait/motor coordination to traverse a rough terrain
Using a snake robot as a platform to study the locomotion and the motor coordination of a actual snake inorder to traverse a rough terrain or a changing environment. A snake has large potential to move in various environments by drastically changing its 'gait' pattern, in spite of its simple body. The prime aim is to device a control scheme for the snake robot and develop a learning algorithm to acquire a good control rule in a changing environment. 

# Design of the Snake robot:
The Snake robot is designed to be modular and capable of performing various gaits of an actual snake. The robot consists of several segments connected serially. Each module encloses a servo which is oriented 90 degrees in respect to the previous servo axis which makes it mobile in both the ground plane and the plane vertical to it. Thereby making the robot capable of traversing any challenging environment.

# Locomotion control:
To device a learning mechanism to find a proper combination of control parameters for goal-directed locomotion of the robot. Policy Improvement with Path Integrals (PI2) as a learning mechanism has already been implemented, the goal is to formulate another kind of policy formation which integrates well with the kinematic model of the robot.

# Learning parameters:
The key parameters being the velocity and the joint angles of each module.

# Deep-Q-learning
-yet-to-implement-
