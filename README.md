## Ergodic Search Agent
**Author: Jared Berry**

This project was associated with Northwestern University ME 455: Active Learning (Spring 2025).

#### Objective
The simulation environment was created by Max Muchen Sun, as a project template for MECH_ENG 455: Active Learning. It contains a sensor agent (robot) with 2d single integrator dynamics (controlled by velocity), a randomly generated box hidden from the robot, and a built-in prediction function that utilizes generative modeling techniques to predict possible box locations and dimensions given sensor reading history. 

The goal of this project was to control a robot to move across a space to collect signal measurements, such that the uncertainty (variance) of the box predictions drops below a predefine threshold as quickly as possible.

#### Algorithm Overview
Step 1: Check positive sensor reading threshold and determine search state.

    Step 2: Turn predicted boxes into a "target" distribution
      
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;a: Use negative/positive sensor readings (based on search state)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;b: Create pdf based on sample locations with Kernel Density Estimation (KDE)

Step 3: Recalculate Fourier coefficients of target distribution

Step 4: Plan control signal for next time step based on search state
      
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;a: Positive sensor reading threshold not reached

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;i. If an ergodic trajectory isn't already planned, use iLQR to plan one. 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;ii. If ergodic trajectory is planned, follow it.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;b: Positive sensor reading threshold reached

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;i. If trajectory isn't planned, plan route to corner with most amount of variance in
               its possible location.

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;ii. If trajectory is planned, follow it.

Step 5: Save planned control signal and send to gym simulation function.