% Heading autopilot
clear all
clc

%% Initializing
tstart=0;      %Sim start time
tstop=3000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

n_c = 7.3;
delta_c = 0;

%% Task 1.2
%finding K and T for the nomoto model
task_1_2

%testing nomoto performance
omega_d = 0.008;
amp = 0.3;
sim NomotoModelTask12
sim MSFartoystyringtask122

figure()
plot(t,r)
hold on
plot(t,nomotoresult,'r--')
grid on
xlabel('Time [s]')
ylabel('Yaw rate [rad]')
legend('MS Fartøystyring','1st order Nomoto model')
title('Nomoto performance vs full system performance')

%% Task 1.4
omega_n = 10*omega_d;

%controller gains
K_p_heading = omega_n^2*T/K;
K_d_heading = (2*omega_n*T-1)/K;
K_i_heading = omega_n/10*K_p_heading;

c=1; %current on
sim MSFartoystyringTask14

%plotting
figure()
plot(t,psi_tilde)
grid on
xlabel('Time [s]')
ylabel('Heading error [rad]')
legend('psi_{tilde}');
title('Closed-Loop Behaviour of psi_{tilde}')

figure()
plot(t,psi,t,psi_d,'--r')
grid on
xlabel('Time [s]')
ylabel('Heading [rad]')
legend('psi','psi_d');
title('Closed-Loop Behaviour of psi and psi_d')

figure()
plot(t,r_tilde)
grid on
xlabel('Time [s]')
ylabel('Heading rate error [rad/s]')
legend('r_{tilde}');
title('Closed-Loop Behaviour of r_{tilde}')

figure()
plot(t,r,t,r_d,'--r')
grid on
xlabel('Time [s]')
ylabel('Heading rate [rad/s]')
legend('r','r_d');
title('Closed-Loop Behaviour of r and r_d')
