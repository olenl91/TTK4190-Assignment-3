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
%% Task 2.7
k = 0; %used to start at first waypoint in headingGuidance.m
c=1; %current on

%Initializing controllers
initcontrollers

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
