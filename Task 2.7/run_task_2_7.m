%Part 2.1,2.2,2.3
clear all
clc

%% Initializing
tstart=0;      %Sim start time
tstop=7000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)


p0=zeros(2,1); %Initial position (NED)
u_r_0 = 6.63;
v0=[u_r_0 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

global e_d U_t delta_s delta_e chi_t p_t kappa R_p s_d;

%% Loading waypoints
load('WP.mat')
WP_xpos = WP(1,:);
WP_ypos = WP(2,:);
WP_time = [0 40 60 80 100]; %as in the example from p.269 in [1]

%% Task 2.7
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
omega_heading = 0.10;
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
K_windup_speed = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

L = 304.8; %length of ship (m)
U_t = 3;
e_d = 400;
chi_t = 36.31*pi/180;
p0_t = [4860;3880];
p_t = p0_t;
kappa = 4;
delta_e = 2*L;
delta_s = 500;
R_p = [cos(chi_t)  -sin(chi_t); sin(chi_t) cos(chi_t)];
s_d = 0;

sim MSFartoystyringtask27

%Simulation results
pathplotter(p(:,1), p(:,2), psi, tsamp, 10, tstart, tstop, 1, WP)

%Plotting
%add plots for closed loop system
