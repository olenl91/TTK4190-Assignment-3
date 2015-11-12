%Part 2.1,2.2,2.3
clear all
clc

%% Initializing
tstart=0;      %Sim start time
tstop=2800;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)


p0=zeros(2,1); %Initial position (NED)
u_r_0 = 6.63;
v0=[u_r_0 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

global k WP_xpos WP_ypos R_k1 K_p_guidance;

%% Loading waypoints
load('WP.mat')
WP_xpos = WP(1,:);
WP_ypos = WP(2,:);
WP_time = [0 40 60 80 100]; %as in the example from p.269 in [1]

%% Task 2.3
k = 0; %used to start at first waypoint in headingGuidance.m
m_u = 6000;
c=1; %current on

%Initializing controllers
initcontrollers

%Heading guidance parameters
L = 304.8; %length of ship (m)
R_k1 = 2*L; %Circle of acceptance


sim MSFartoystyringtask26

%Simulation results
pathplotter(p(:,1), p(:,2), psi, tsamp, 10, tstart, tstop, 0, WP)


%Plotting

