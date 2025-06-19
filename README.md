## Ergodic Search Agent
**Author: Jared Berry**

This project was associated with Northwestern University ME 455: Active Learning (Spring 2025).

#### Objective
The simulation environment was created by Max Muchen Sun, as a project template for MECH_ENG 455: Active Learning. It contains a sensor agent (robot) with 2d single integrator dynamics (controlled by velocity), a randomly generated box hidden from the robot, and a built-in prediction function that utilizes generative modeling techniques to predict possible box locations and dimensions given sensor reading history. 

The goal of this project was to control a robot to move across a space to collect signal measurements, such that the uncertainty (variance) of the box predictions drops below a predefine threshold as quickly as possible.

#### Algorithm Description
