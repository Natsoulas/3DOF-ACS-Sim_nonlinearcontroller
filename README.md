# 3DOF-ACS-Sim_nonlinearcontroller
3 DOF Attitude Control Simulation of a spacecraft using a Reaction thruster control system.

Main file to run is called "Control_Loop_ODE45.m", you can specify what attitude you desire, and you may specify/append an attitude to track with an external dynamics sim. You may also edit thruster data files and tables such that they match the spacecraft you are simulating better. Typical thruster data matrix or table is each row is a thruster with prescribed angle along spacecraft bus, or xyz body coordinates, and with a sizing of thrust force which is allocated as the last column in my use. There is also the body xyz cartesian pointing direction for the given force of a thruster.

Augmented optimization can be further toyed with in the function script "simplex_runner.m" by playing with cost function and orders of magnitude of penalty for each consideration (prop use, thruster activity etc.)

You may use thruster_placement.m to help size and place thrusters along a spacecraft bus. Keep in mind this will output a string matrix that you need to convert to a double to use properly in the simulation.

There are quite a few test files for testing different components standalone which is what I used for debugging during development.

Also, I use quaternion in the state such that the scalar term is q0 or the first element of the quaternion, and the last three are the vector component. Some people do it backwards :(...

Built this myself, however the simplex optimization augmentation is the intellectual property of Brent Faller.
