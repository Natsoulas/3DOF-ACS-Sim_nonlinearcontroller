clc
close all
clear all
%Define IC's
%(junkbox)==============================================================================================
%%%Describe spacecraft geometry/inertia/thrusters
IC.MOI = diag([8691,28100,34911]); % moment of inertia matrix for the spacecraft
IC.thrusterdata = load('thruster_plac_data.mat');
IC.R = 1;
IC.thrusterdata = IC.thrusterdata.Thruster_PlacementData;
IC.thrusterdata(:,8) = 25;
IC.CG = [1.25;0.1;0]; %center of gravity/center of mass of spacecraft
%%%Rotational dynamics parameters
IC.N_C_B = eye(3); % DCM for body to inertial: identity matrix to say they are the same initially
IC.N_C_R = rotz(90); % DCM for ref to inertial: 90 degree rotation matrix about the z-axis
IC.qr0_B = dcm2quat(inv(IC.N_C_B)*IC.N_C_R);
IC.w_bn0 = transpose([0 0 0]);
IC.qb0_N = transpose([1 0 0 0]);
%%%reference------------------------------------------
ref.w_rn_R = [0;0;0]; %reference angular elocity of r/n in R frame
ref.wdotrn_R = [0;0;0]; %time derivative of element above
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
z = z0; %intializes z as z0
%log z (state)
z_log = [];
%%%reformatting thruster data (ignore pls)
thrusterin4simplex = [];
for k = 1:1:size(IC.thrusterdata,1)
        angle = str2double(IC.thrusterdata(k,3));
        [X,Y] = azim2cartbody(angle,IC.CG,IC.R);
        h = str2double(IC.thrusterdata(k,4));
        ZwrtCOM = h - IC.CG(3);
        Z = ZwrtCOM;
        U = str2double(IC.thrusterdata(k,5));
        V = str2double(IC.thrusterdata(k,6));
        W = str2double(IC.thrusterdata(k,7));
        force = str2double(IC.thrusterdata(k,8));
        thrusterin4simplex = [thrusterin4simplex; k,X,Y,Z,U,V,W,force];
end
F_ref = [0;0;0]; %no translational force for now please
%======================================================================================================
%establish control loop
for t_seconds = 0:1:3600
    %starts with parameters fed into attitude controller
    %z = z0 for sake of input
    L_command = attitude_controller(z,IC,ref);
    % simplex
    solution = simplexrunner(thrusterin4simplex,L_command,F_ref,IC.CG);
    % PWPF
    U_s = zeros(size(solution,1),1000);
    for yi = 1:1:size(solution,1)
    t_s = solution(yi,1);
    t_s = round(t_s*1000)/1000;%rounds seconds to nearest millisecond
    if t_s > 1.00
        t_s = 1.00;
    end
    t.microsteps = round(t_s/0.001); %tells number of milliseconds which makes the stepsize 1 millisecond
    if t_s ~= 0
        [u,DC,f_o] = PWPF_Run(IC.C,IC.K_p,IC.K_m,IC.T_m,IC.U_on,IC.U_off,t_s,t.microsteps,IC.tol);
        for ji = 1:size(u,2)
            U_s(yi,ji) = u(ji);
        end
    end
%     disp('DC')
%     disp(DC)
%     disp('f_o')
%     disp(f_o)
    end
    % PWPF output is then processed into torques given thruster data
    %%bite of foreign code
        %compute total torque vectors for all unique sets of n-thrusters
    dimlong = size(IC.thrusterdata,1);
    thrusterno = 1:dimlong;
    Torque_ms = zeros(3,1000);
    for millisec = 1:1000
        %loop through which millisecond you are on.
        Tmilli = [0;0;0];
        for k = 1:dimlong
            angle = str2double(IC.thrusterdata(k,3));
            [X,Y] = azim2cartbody(angle,IC.CG,IC.R);
            h = str2double(IC.thrusterdata(k,4));
            ZwrtCOM = h - IC.CG(3);
            Z = ZwrtCOM;
            r = [X;Y;Z];
            U = str2double(IC.thrusterdata(k,5));
            V = str2double(IC.thrusterdata(k,6));
            W = str2double(IC.thrusterdata(k,7));
            force = str2double(IC.thrusterdata(k,8));
            F = force*[U;V;W];
            if U_s(k,millisec) == 1
                Tconcat = cross(r,F);
            else
                Tconcat = [0;0;0];
            end
            %sums all the produced torques for the particular millisecond
            %but only if the partifular thruster is on.
            Tmilli = Tmilli + Tconcat;
        end
        Torque_ms(:,millisec) = Tmilli;
    end
    %%
    % torques summed and fed into ode5
    IC.T_control = Torque_ms;
    t.span = 1;
    t.steps = 1000;
    res = attitude_dynamics_modelode5(t,z,IC);
    z_update_2controller = res(end,:);
    % ode5 sends state back through loop for controller to deal with
        %updates state variables in state row vector z
    z_log = [z_log; res];
    z = z_update_2controller;
    %t.span = 3600;
    %t.steps = 3600;

end