%dynamics model test script
clc
close all
clear all
IC.MOI = diag([8691,28100,34911]);
IC.T_control = [10000;0;0];
euler = [0 pi/2 0];
t.span = 100;
t.steps = 2500;
quat = Euler3212EP(euler);
z0 = [quat(1),quat(2),quat(3),quat(4),0,0,0];
res = attitude_dynamics_modelode5(t,z0,IC);