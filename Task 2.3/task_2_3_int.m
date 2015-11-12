%Part 2.3 with integral action
clear all

%% Initializing
tstart=0;      %Sim start time
tstop=3000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)


p0=zeros(2,1); %Initial position (NED)
u_r_0 = 6.63;
v0=[u_r_0 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

global k WP_xpos WP_ypos R_k1;

%% Task 2.1
%Loading waypoints
load('WP.mat')
WP_xpos = WP(1,:);
WP_ypos = WP(2,:);

%% Task 2.3
k = 0; %used to start at first waypoint in headingGuidance.m
m_u = 6000;
c=1; %current on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initializing controllers%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Nomoto model
T = 74;
K = -0.0272;
%Heading
omega_n = 0.08;
K_p_heading = omega_n^2*T/K;
K_d_heading = (2*omega_n*T-1)/K;
K_i_heading = omega_n/10*K_p_heading;
delta_c_max = 25*pi/180; % Rudder saturation
%Heading reference model
zeta_heading = 1;
omega_heading = 0.05;
%Forward speed model
d_1 = -0.0634;
d_2 = 1.0463;
%Speed
lambda = 0.23;
K_p_speed = 2*lambda;
K_i_speed = lambda^2;
n_c_max = 8.9; % Shaft speed saturation
%Speed reference model
zeta_speed = 1;
omega_speed = 0.01;
%Integral windup - anti-windup gains
K_windup_heading = 1;
K_windup_speed = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Heading guidance parameters
L = 304.8; %length of ship (m)
R_k1 = 2*L; %Circle of acceptance
delta_guidance = 2*L; %Look-ahead distance

%Saturating control law with integral action simulation
%Guidance control gains
K_p_guidance = 1/delta_guidance;
K_i_guidance = 1/(250*(delta_guidance));
sim MSFartoystyringtask23int
%Simulation results
pathplotter(p(:,1), p(:,2), psi, tsamp, 10, tstart, tstop, 0, WP)
title('Saturating control with integral action simulation')

%% Plotting closed loop behaviour
figure()
plot(t,delta_c)
hold on
plot([t(1),t(end)],[delta_c_max,delta_c_max],'--k',[t(1),t(end)],[-delta_c_max,-delta_c_max],'--k')
grid on
xlabel('Time [s]')
ylabel('Rudder Command Angle [rad]')
title('Closed-Loop Behaviour of Heading Controller with integral action')
legend('delta_c','delta_{sat}');

figure()
plot(t,psi,t,psi_r,'black:',t,psi_d,'r--')
grid on
xlabel('Time [s]')
ylabel('Heading [rad]')
title('Closed-Loop Behaviour of Heading Controller with integral action')
legend('psi','psi_r','psi_d');

figure()
plot(t,r,t,r_d,'r--')
grid on
xlabel('Time [s]')
ylabel('Heading rate [rad/s]')
title('Closed-Loop Behaviour of Heading Controller with integral action')
legend('r','r_d');

figure()
plot(t,n_c,[t(1),t(end)],[n_c_max,n_c_max],'--k');
grid on
xlabel('Time [s]')
ylabel('Shaft Command Speed [m/s]')
title('Closed-Loop Behaviour of Speed Controller with integral action')
legend('n_c','n_{max}');

figure()
plot(t,v(:,1),t,u_r,'black:',t,u_d,'r--')
grid on
xlabel('Time [s]')
ylabel('Forward Speed [m/s]')
title('Closed-Loop Behaviour of Speed Controller with integral action')
legend('u','u_r','u_d','Location','SouthEast');
