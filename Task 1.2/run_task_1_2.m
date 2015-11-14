tstart=0;      %Sim start time
tstop=2000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=0;           %Current on (1)/off (0)

nc = 7;
dc = 10/180*pi; % 10 degree turning angle

sim MSFartoystyring

% find the terminal yaw rate and velocity
r_terminal = r(size(r, 1));
v_terminal = sqrt(v(size(v,1),1)^2 + v(size(v,1),2)^2);

K = r_terminal/dc;

% matlab approximation function
x0 = [100 0.1]';
F = @(x, t) r0*exp(-t/x(1)) + (1 + -exp(-t/x(1)))*(K*dc); % x = [T]
x = lsqcurvefit(F,x0, t', r');
T = x(1);


figure(); hold on;
title('Speed plot over simulation time')
plot(t, sqrt(v(:,1).^2 + v(:,2).^2)); % plots speed against time to see when they reach terminal
xlabel('Time [s]')
ylabel('Velocity [m/s]')
figure(); hold on;
title('yaw rate over simulation time compared to nomoto model')
xlabel('Time [s]')
ylabel('Yaw rate [rad/s]')
plot(t, r); % plot yaw rate to see when it reach terminal rate
plot(t, F([T -K], t)); % plotting the Nomoto model
legend('simulation','nomoto model');
figure(); hold on;
title('Position plot of the turning circle')
plot(p(:,1), p(:,2)); % plot the position in the plane

