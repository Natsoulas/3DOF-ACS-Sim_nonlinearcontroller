clc
close all
clear all
%standalone controller sim
%parameters
IC.MOI = diag([8691,28100,34911]); % moment of inertia matrix for the spacecraft
IC.thrusterdata = load('thruster_plac_data.mat');
IC.R = 1;
IC.thrusterdata = IC.thrusterdata.Thruster_PlacementData;
IC.CG = [0;0;3]; %center of gravity/center of mass of spacecraft
%%%Rotational dynamics parameters
IC.N_C_B = eye(3); % DCM for body to inertial: identity matrix to say they are the same initially
IC.N_C_R = rotz(90); % DCM for ref to inertial: 90 degree rotation matrix about the z-axis
IC.qr0_B = dcm2quat(inv(IC.N_C_B)*IC.N_C_R);
IC.w_bn0 = transpose([0 0 0]);
IC.qb0_N = transpose([1 0 0 0]);
%%%reference------------------------------------------
ref.w_rn_R = [0;0;0]; %unsure about this ask Brent
ref.wdotrn_R = [0;0;0];
%%%----------------------------------------------------
%delta---updates throughout loop
IC.delta_w_bn0 = IC.w_bn0 - inv(IC.N_C_B)*IC.N_C_R*ref.w_rn_R;
%external torques (gravity, drag, etc.)
IC.L_external = 0; %external torques in EOM (0 for now)
%gains for controller and pwpf modulator
IC.K = 15; %check boulder slides
IC.P_matrix = IC.K*400*eye(3);
IC.C = 1; %pwpf command signal (set at 1 to accept burn durations from simplex)
IC.K_p = 1; % proportional tuning gain for pwpf
IC.K_m = 4.5; % tuning gain for pwpf
IC.T_m = 0.85; %tuning gain for pwpf
IC.U_on = 0.8; %tuning parameter for pwpf (schmitt trigger)
IC.U_off = IC.U_on/9; %tuning parameter for pwpf (schmitt trigger)
%tolerance for Schmitt Trigger and time structure for numint
IC.tol = 0.004; %tolerance
t.span = 3600;
t.steps = 3600;
t.microsteps = 1000;
t.microspan = 1;
%%% Initial state
z0 = [dcm2quat(IC.N_C_B),0,0,0];
z = z0;


[L_command,sig,quat] = attitude_controller(z,IC,ref);