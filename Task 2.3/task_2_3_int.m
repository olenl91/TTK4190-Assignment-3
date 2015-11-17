%task 2.3 with integral action
clear all
clc

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

%Loading waypoints
load('WP.mat')
WP_xpos = WP(1,:);
WP_ypos = WP(2,:);

k = 0; %used to start at first waypoint in headingGuidance.m
c=1; %current on

%Initializing controllers
initcontrollers

%Heading guidance parameters
L = 304.8; %length of ship (m)
R_k1 = 2*L; %Circle of acceptance
K_i_guidance = 1/300; %1/300*K_p_guidance

%% Lookahead-based steering with PI-controller simulation
int_on = 1; %integral action on
sim MSFartoystyringtask23
%Simulation results
pathplotter(p(:,1), p(:,2), psi, tsamp, 10, tstart, tstop, 0, WP)
title('Simulated path with PI-controller')
