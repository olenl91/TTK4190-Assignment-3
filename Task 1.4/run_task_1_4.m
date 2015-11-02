tstart=0;      %Sim start time
tstop=1000;    %Sim stop time
tsamp=10;      %Sampling time (NOT ODE solver time step)

p0=zeros(2,1); %Initial position (NED)
v0=[6.63 0]';  %Initial velocity (body)
psi0=0;        %Inital yaw angle
r0=0;          %Inital yaw rate
c=1;           %Current on (1)/off (0)



m = 1/0.0003927;
T = 1/0.006673;




zeta=1; 
Omega_b = 0.09; 
Omega_n = (1/(sqrt(1-2*zeta^2+sqrt(4*zeta^4-4*zeta^2+2))))*Omega_b;
K_p = (m)*Omega_n^2;
K_d = 2*zeta*Omega_n*(m)-m/T;
K_i = (Omega_n/10)*K_p;

sim MSFartoystyring





figure(1)
plot(t,psi*180/pi,'.',t,-0.3*sin(0.008*t)*180/pi)
xlabel('Time [s]')
ylabel('Heading [deg]')
legend('\psi','\psi_d')

figure(2)
plot(t,r*180/pi,'.',t,-0.0024*cos(0.008*t)*180/pi)
xlabel('Time [s]')
ylabel('r [deg/s]')
legend('r','r_d')

figure(3)
plot(t,rerror*180/pi)
xlabel('Time [s]')
ylabel('Heading rate error [deg/s]')
legend('r_e')

figure(4)
plot(t,psi_e*180/pi)
xlabel('Time [s]')
ylabel('Heading error [deg]')
legend('\psi_e')