%Speed controller
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

%Heading controller init
omega_d = 0.008;
omega_n = 10*omega_d;
T = 59.13;
K = -0.0252;
K_p = omega_n^2*T/K;
K_d = (2*omega_n*T-1)/K;
K_i = omega_n/10*K_p;

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

m_u = 6000;
n_c = n_c_1;
sim MSFartoystyringtask16
sim ForwardSpeedModel

figure(1);
plot(t,v(:,1),t,u,'--r')
grid on
xlabel('Time [s]')
ylabel('Forward Speed [m/s]')
legend('MS Fartøystyring','Forward Speed Model')
title('Step response comparison')

%% Task 1.8
u_r_0 = 4;
u_r_step = 7;
stepTime = 700;
v0=[u_r_0 0]';
c = 1; %Current on

%Speed controller gains
lambda = 0.23;
K_p_speed = 2*lambda;
K_i_speed = lambda^2;

%Speed reference model parameters
zeta_speed = 1; %critical damping
omega_speed = 0.01;

sim MSFartoystyringtask18

figure(2)
plot(t,u_tilde);
grid on
xlabel('Time [s]')
ylabel('Forward Speed Error [m/s]')
legend('u_{tilde}');
title('Closed-Loop Behavior of u_{tilde}')

figure(3)
plot(t,v(:,1),t,u_d,'r--',t,u_r,'black:')
grid on
xlabel('Time [s]')
ylabel('Forward Speed [m/s]')
axis([0 tstop 3.5 7.5])
legend('u','u_d','u_r');
title('Closed-Loop Behavior of u, u_d and u_r')
