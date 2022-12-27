# Motor_Control

Designed a LQR controller to fulfill position control of a motor.
The main executive file is final_LQR_zero_assignment.cpp which envolves the designed controller and code to drive the motor.

In LQR_Design.m file, you can find the state space representation of the motor and a controller designed based on LQR control law. Plus, the closed-loop step response is also simulated.

In compensator_analysis.m file, you can find step responses of a controller input, error, and closed-loop response for further analysis.

In robustness_check.m file, a simple algorithm to test the robustness of a controller is implemented. In this file, the parameters of the motor vary within a certain range, and the corresponding closed-loop step responses are simulated to verify stability.


The two slx file implement everything described above in simulink environment for better visualization. 

