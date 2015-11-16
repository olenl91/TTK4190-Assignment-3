%plotting for task 2.3
figure()
plot(t,delta_c)
hold on
plot([t(1),t(end)],[delta_c_max,delta_c_max],'--k',[t(1),t(end)],[-delta_c_max,-delta_c_max],'--k')
grid on
xlabel('Time [s]')
ylabel('Rudder Command Angle [rad]')
title('Closed-Loop Behaviour of Heading Controller - Integral action')
legend('delta_c','delta_{sat}');

figure()
plot(t,psi,t,psi_r,'black:',t,psi_d,'r--')
grid on
xlabel('Time [s]')
ylabel('Heading [rad]')
title('Closed-Loop Behaviour of Heading Controller - Integral action')
legend('psi','psi_r','psi_d');

figure()
plot(t,r,t,r_d,'r--')
grid on
xlabel('Time [s]')
ylabel('Heading rate [rad/s]')
title('Closed-Loop Behaviour of Heading Controller - Integral action')
legend('r','r_d');

figure()
plot(t,n_c,[t(1),t(end)],[n_c_max,n_c_max],'--k');
grid on
xlabel('Time [s]')
ylabel('Shaft Command Speed [m/s]')
title('Closed-Loop Behaviour of Speed Controller - Integral action')
legend('n_c','n_{max}');

figure()
plot(t,v(:,1),t,u_r,'black:',t,u_d,'r--')
grid on
xlabel('Time [s]')
ylabel('Forward Speed [m/s]')
title('Closed-Loop Behaviour of Speed Controller - Integral action')
legend('u','u_r','u_d','Location','SouthEast');
