%PWPF Technical Paper figure replication
%from Trond Dagfinn Krovel's Paper: "Optimal Selection of PWPF Parameters"
C = 0.5; %constant input value
K_p = 1; %Pre-modulator gain
K_m = 4.5; %Modulator gain
T_m = 0.85; %Modulator time constant
U_on = 0.45; %Shmitt trigger on-value
U_off = U_on/3; %Shmitt trigger off-value
h = U_on - U_off;

t_s = 0.6; %Set the simulation time-span


%f_of_t = fzero + (K_m*(C-U)-fzero)*(1 - exp(-t/T_m));
%t = linspace(0,t_s,100);
 %initialize f(0) and U (u_of_t)
 U = 1;
 T_on = -T_m*log(1-(h/(U_on-K_m*(C-U))));
 %for t = 0:0.005:0.60
 T_off = -T_m*log(1-(h/(K_m*C-U_off)));  
 %end
 for t = 0:0.005:0.6
    
 end