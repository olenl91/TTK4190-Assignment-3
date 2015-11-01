% addpath('../code');

tstart=0;      %Sim start time
tstop=1;    %Sim stop time
tsamp=0.01;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[0.000000001 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

nc = 7; % the actuation of the propeller
dt = 1; % how many timesteps for evaluating u dot

sim('MSFartoystyring');

% calculate mk = m/K_n based on u ~= 0

u = sqrt(v(:,1).^2 + v(:,2).^2);
udot = (u(1+dt) - u(1))/(t(1+dt) - t(1));

mk = 1/udot*nc*nc;



% do the plotting of u with u dot estimation
figure;
hold on;
plot(t, u);
axis([-t(1+dt) 2*t(1+dt) -u(1+dt) 2*u(1+dt)]);
%plot(t(1),u(1),'*',t(1+dt),u(1+dt),'*'); % set the points for the calculation
plot([t(1) t(1+dt) t(1+dt)],[u(1) u(1) u(1+dt)]); % the lines showing the acceleration estimate

