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
delta_step = 12.5*pi/180; %12.5degrees delta_c step input
sim MSFartoystyringtask12step

delta_r = 0.632*(r(end)-r(1))+r(1);

T = 59.13;

%plotting step response
figure(1)
plot(t,r,T,delta_r,'or')
grid on
xlabel('Time [s]')
ylabel('Yaw rate [rad]')
legend('Step response','63.2% of \Deltar')
title('Yaw Rate Step Response')

K = (r(end)-r(1))/delta_step;

%testing nomoto performance
omega_d = 0.008;
amp = 0.3;
sim NomotoModelTask12
sim MSFartoystyringtask12sine

figure(2)
plot(t,r)
hold on
plot(t,nomotoresult,'r--')
grid on
xlabel('Time [s]')
ylabel('Yaw rate [rad]')
legend('MS Fartøystyring','1st order Nomoto model')
title('Nomato performance vs full system performance, original K')

% %improved Nomoto performance
K = 0.8*(r(end)-r(1))/delta_step;
sim NomotoModelTask12

figure(3)
plot(t,r)
hold on
plot(t,nomotoresult,'r--')
grid on
xlabel('Time [s]')
ylabel('Yaw rate [rad]')
legend('MS Fartøystyring','1st order Nomoto model')
title('Nomato performance vs full system performance, improved performance')

%% Task 1.4
omega_n = 10*omega_d;

%controller gains
K_p = omega_n^2*T/K;
K_d = (2*omega_n*T-1)/K;
K_i = omega_n/10*K_p;

c=1; %current on
sim MSFartoystyringTask14

%plotting
figure(4)
plot(t,psi_tilde)
grid on
xlabel('Time [s]')
ylabel('Heading error [rad]')
legend('psi_{tilde}');
title('Closed-Loop Behaviour of psi_{tilde}')

figure(5)
plot(t,psi,t,psi_d,'--r')
grid on
xlabel('Time [s]')
ylabel('Heading [rad]')
legend('psi','psi_d');
title('Closed-Loop Behaviour of psi and psi_d')

figure(6)
plot(t,r_tilde)
grid on
xlabel('Time [s]')
ylabel('Heading rate error [rad/s]')
legend('r_{tilde}');
title('Closed-Loop Behaviour of r_{tilde}')

figure(7)
plot(t,r,t,r_d,'--r')
grid on
xlabel('Time [s]')
ylabel('Heading rate [rad/s]')
legend('r','r_d');
title('Closed-Loop Behaviour of r and r_d')
