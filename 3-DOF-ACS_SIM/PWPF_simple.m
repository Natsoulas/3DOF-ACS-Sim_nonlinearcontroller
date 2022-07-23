%pwpf matlab (from simulink design)
%model based off of Trond Dagfinn Krovel's Paper: "Optimal Selection of PWPF Parameters"
function u = PWPF_simple(C,K_p,K_m,T_m,U_on,U_off,t_s,timesteps)
%initialize
fzero = 0;
U_initial = 0;
f = [];
u = [];
r = C*ones(1,timesteps);
uconcat = U_initial;
u = [u uconcat];
counter = 1;
for t = 0:timesteps:t_s
f_time = fzero + (K_m*(K_p*r(counter)-u(counter))-fzero)*(1 - exp(-t/T_m));
f =[f f_time];
if f(counter) < U_off
    uconcat = 0;
elseif f(counter) == U_on
    uconcat = 1;
elseif uconcat == U_off
    uconcat = 0;
elseif counter == timsteps
    uconcat = [];
else
    uconcat = u(counter-1);
end
u = [u uconcat];
counter = counter + 1;
end
t = linspace(0,t_s,timesteps);
u = u(1:end-1);
figure
plot(t,r,t,f,t,u)
end