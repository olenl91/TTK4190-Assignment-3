%Part 2.7
clear all
clc

%% Initializing
tstart=0;      %Sim start time
tstop=7500;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
u_r_0 = 6.63;
v0=[u_r_0 0]'; %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

global k WP_xpos WP_ypos R_k1 U_t u_max Kappa delta_speed desired_distance;

%% Loading waypoints
load('WP.mat')
WP_xpos = WP(1,1:2);
WP_ypos = WP(2,1:2); %First two waypoints for tracking

%% Task 2.7
k = 0; %used to start at first waypoint in headingGuidance.m
c=1; %current on

%Initializing controllers
initcontrollers

%Tracking
U_t = 3; %Target speed
u_max = 7.2607; %Max speed
startpos_x = WP(1,2);
startpos_y = WP(2,2);
tracking_angle = atan2(WP(2,2)-WP(2,1),WP(1,2)-WP(1,1));

%Heading guidance
L = 304.8; %length of ship (m)
R_k1 = 2*L; %Circle of acceptance
K_i_guidance = 1/300; %1/300*K_p_guidance
int_on = 0; %Integral action off

%Speed guidance
Kappa = u_max - U_t; %maximum approach speed
delta_speed = 2000;
desired_distance = 100; %m

%Simulate
sim MSFartoystyringtask27

%Simulation results
pathplotter(p(:,1), p(:,2), psi, tsamp, 10, tstart, tstop, 1, WP)
plots