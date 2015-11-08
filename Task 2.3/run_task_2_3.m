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

global k WP_xpos WP_ypos R_k1;

%% Task 2.1
%Loading waypoints
load('WP.mat')
WP_xpos = WP(1,:);
WP_ypos = WP(2,:);
WP_time = [0 40 60 80 100]; %as in the example from p.269 in [1]

%Finding ship turning radius
sim MSFartoystyringtask21
pathplotter(p(:,1),p(:,2),psi,tsamp,1000,tstart,tstop,0,zeros(2,tstop))
turningRadius = 800; %m

%Method 1 - Continous interpolation
t = 0:1:max(WP_time);
x_spline = spline(WP_time, WP_xpos,t); %Spline inter
y_spline = spline(WP_time, WP_ypos,t);
x_pchip = pchip(WP_time, WP_xpos,t); %Hermite inter
y_pchip = pchip(WP_time, WP_ypos,t);

%Method 2 - Piece-wise continous interpolation
x_spline2 = spline(WP_time, WP_xpos);
y_spline2 = spline(WP_time, WP_ypos);
x_p = ppval(x_spline2, WP_time);
y_p = ppval(y_spline2, WP_time);

%Plotting method 1 +2 
figure()
plot(WP_ypos, WP_xpos,'o') %plot waypoints
hold on;
plot(y_p,x_p,'r') %plot method 2
hold on;
plot(y_spline,x_spline, 'g') %plot method 1 - spline
hold on;
plot(y_pchip,x_pchip, 'black') %plot method 1 - pchip
hold on;
axis([-6000 12000 -2000 16000])

%Method 3 - Circles and straight lines - Dubins path
line(WP_ypos,WP_xpos)
for i = 2 : 4
    lastWP = [WP(1,i-1); WP(2,i-1)];
    currentWP = [WP(1,i); WP(2,i)]; %waypoints
    nextWP = [WP(1,i+1); WP(2,i+1)];
    u = (lastWP - currentWP)/(norm(lastWP-currentWP));
    v = (nextWP - currentWP)/(norm(nextWP-currentWP)); %a,b,c
    w = (u + v)/(norm(u + v));
    a = acos((norm(v)^2 + norm(u+v)^2 - norm(u)^2)/(2*norm(v)*norm(u+v)));
    R = turningRadius / sin(a);
    circlePos = currentWP + (R * w);
    viscircles([circlePos(2,1), circlePos(1,1)], turningRadius, 'LineWidth', 1);
    hold on;
end
legend('Waypoints','Dubins path','Spline inter.','Cubic Hermite inter.','Piece-wise inter.');
xlabel('y - East [m]');
ylabel('x - North [m]');
title('Task 2.1 - path generation plot')

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
omega_heading = 0.15;
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

%Heading guidance parameters
L = 304.8; %length of ship (m)
R_k1 = 2*L; %Circle of acceptance
delta_guidance = 2*L; %Look-ahead distance

%Guidance control gains
K_p_guidance = 1/delta_guidance;
K_i_guidance = 1/(250*(delta_guidance));

%do some tuning...

sim MSFartoystyringtask23

%Simulation results
pathplotter(p(:,1), p(:,2), psi, tsamp, 10, tstart, tstop, 0, WP)
%fix pathplotter error

%Plotting
%add plots for closed loop system
