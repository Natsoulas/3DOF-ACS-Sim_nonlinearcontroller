%script interface for thrusterplacement---simplex---pwpf
clc
close all
clear all
%DATA inputs===============================================================
%for thruster placement tool
COM = [0,0,3];
R = 1;
config14 = [1,0,0,1,0;2,0,0,-1,0;3,0,0,0,1;4,0,0,0,-1;5,90,1,0,0;6,90,-1,0,0;7,90,0,0,1;8,90,0,0,-1;9,180,0,1,0;10,180,0,-1,0;11,180,0,0,1;12,180,0,0,-1;13,270,1,0,0;14,270,-1,0,0;15,270,0,0,1;16,270,0,0,-1];
Thruster_PlacementData = thrusterplacement([],3, config14, [2,4,6], [2,0,0;0,3,0;0,0,1], R, COM, 10,"none","none","none","none");
% It is also an option just to import a predefined thruster_placementdata
% matrix as it would save on computational time if you don't need to use
% the tool every time you run simplex and pwpf
%==========================================================================
thrusterin4simplex = [];
for k = 1:1:size(Thruster_PlacementData,1)
        angle = str2double(Thruster_PlacementData(k,3));
        [X,Y] = azim2cartbody(angle,COM,R);
        h = str2double(Thruster_PlacementData(k,4));
        ZwrtCOM = h - COM(3);
        Z = ZwrtCOM;
        U = str2double(Thruster_PlacementData(k,5));
        V = str2double(Thruster_PlacementData(k,6));
        W = str2double(Thruster_PlacementData(k,7));
        force = str2double(Thruster_PlacementData(k,8));
        thrusterin4simplex = [thrusterin4simplex; k,X,Y,Z,U,V,W,force];
end
%Target torque and translational force for SIMPLEX =======================
L_ref = [20;5;0];
F_ref = [0;10;0];
%=========================================================================
rCG_B = transpose(COM);
solution  = simplexrunner(thrusterin4simplex,L_ref,F_ref,rCG_B);

K_p = 1;
K_m = 4.5;
T_m = 0.85;
U_on = 0.8;
U_off = U_on/9;
%t_s = max(solution);
timesteps = 10000;
tol = 0.002;
C = 1;
U_s = [];
for yi = 1:1:size(solution)
t_s = solution(yi,1);
[u,DC,f_o] = PWPF_Run(C,K_p,K_m,T_m,U_on,U_off,t_s,timesteps,tol);
U_s = [U_s; u];
disp('DC')
disp(DC)
disp('f_o')
disp(f_o)
end

function [X,Y] = azim2cartbody(theta,COM,R)
        Xcom = COM(1);
        Ycom = COM(2);
        Xrel= R*cos(deg2rad(theta));
        Yrel = R*sin(deg2rad(theta));
        XwrtCOM = -Xcom + Xrel;
        YwrtCOM = -Ycom + Yrel;
        X = XwrtCOM;
        Y = YwrtCOM;
end