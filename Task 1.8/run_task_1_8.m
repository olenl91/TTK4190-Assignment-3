%Speed controller
clear all
clc
%% Initializing
tstart=0;      %Sim start time
tstop=4000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

%
p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)


%% Task 1.6
%Finding d1 and d2
n_c_1 = 7.3;
n_c_2 = 5.0; %arbitrarely values, used to find d1 and d2

n_c = n_c_1;
sim MSfartoystyringtask16
u_1 = v(end,1);
n_c = n_c_2;
sim MSfartoystyringtask16
u_2 = v(end,1);

u = [u_1 u_1*abs(u_1);
     u_2 u_2*abs(u_2)];
n = [n_c_1*abs(n_c_1);
     n_c_2*abs(n_c_2)];
d = inv(u)*n;
d_1 = d(1);
d_2 = d(2);


%% Task 1.8