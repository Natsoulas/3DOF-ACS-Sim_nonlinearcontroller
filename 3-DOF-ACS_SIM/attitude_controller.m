function [L_command,sig,quat] = attitude_controller(z,IC,ref)
%takes pwpf modulator gains as input as well as state z for each timestep
%to calculate the control output torque of L_command so that it can be
%propagated through the control loop.
%keep in mind that "w" is angular velocity in the B (body) frame.
%keep in mind that z = [q0 q1 q2 q3 w1 w2 w3];
quat = z(1:4);
IC.N_C_B = inv(quat2dcm(quat)); %I flipped it cuz quat2dcm outputs B_C_N
quat = dcm2quat(inv(IC.N_C_R)*IC.N_C_B);
w = transpose(z(5:7));
wcross = [0,-w(3),w(2);w(3),0,-w(1);-w(2),w(1),0];
sig = quat2mrp(transpose(quat)); %symbol for MRP
%
IC.w_bn0 = w;
%get rid of IC on N_C_B
IC.w_rn_R = ref.w_rn_R;
w_ref_dot = ref.wdotrn_R;
delta_w= IC.w_bn0 - inv(IC.N_C_B)*IC.N_C_R*IC.w_rn_R; %difference between ref and currrent angular velocities - expression from cu boulder slides
L_command = -IC.K*sig - IC.P_matrix*delta_w + IC.MOI*(inv(IC.N_C_B)*IC.N_C_R*w_ref_dot - wcross*inv(IC.N_C_B)*IC.N_C_R*IC.w_rn_R) + wcross*IC.MOI*w - IC.L_external; %expression from cu boulder slides
end