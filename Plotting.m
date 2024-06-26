clear all
Data = load("C:\Users\eliasoa\OneDrive - NTNU\Documents - Modellering, regulatordesign og simulering av kabeldrevet robot\General\TTK4900 Masteroppgave\Rapport\Tester og fremstillig av plots\MATLAB RUN\PlotData.mat");

q           = Data.q_log;
qd          = Data.qd_log;
e           = Data.e_log;  
e_int       = Data.e_int_log;  
f           = Data.f_log;
timeVec     = Data.timeVec;


%% Plot
figure
hold on
plot(q(1,:),q(2,:));


%%

% T_friction  = Data.T_friction_log;

% vel         = Data.vel_log;
% fvel        = Data.fvel_log;
% f_uncon     = Data.f_uncon_log;
% f_friction  = Data.f_friction_log;
% f_friction_signed = Data.f_friction_signed_log;
% w_friction  = Data.w_friction_log;
w_c         = Data.w_c_log;

iterations    = length(q);
tIter = linspace(1,iterations,iterations);

%% Plot of pose and desired pose
close all;
%% x axis: Iteration number
% figure(1)
% subplot(3,1,1)
% plot(tIter , q(1,:), tIter ,qd(1,:));
% title("Plot of x", "Interpreter", "latex")
% legend("x", "$x_d$","Interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(tIter , q(2,:), tIter ,qd(2,:));
% title("Plot of y", "Interpreter", "latex")
% legend("y", "$y_d$","Interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(tIter , rad2deg(q(3,:)), tIter, rad2deg(qd(3,:)) );
% title("Plot of $\phi$", "Interpreter", "latex")
% legend("$\phi$", "$\phi _d$","Interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Angle (deg)", "Interpreter","latex")

% figure(2)
% subplot(3,1,1)
% plot(tIter, e(1,:));
% title("Error in x", "Interpreter", "latex")
% legend("$e_x$","interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(tIter , e(2,:));
% title("Error in y", "Interpreter", "latex")
% legend("$e_y$","interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(tIter , rad2deg(e(3,:)));
% title("Error in $\phi$", "Interpreter", "latex")
% legend("$e_{\phi}$","interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Orientation (deg)", "Interpreter","latex")

% figure(3)
% subplot(3,1,1)
% plot(tIter, e_int(1,:));
% title("Error in x", "Interpreter", "latex")
% legend("$e_x$","interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(tIter , e_int(2,:));
% title("Error in y", "Interpreter", "latex")
% legend("$e_y$","interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(tIter ,(e_int(3,:)));
% title("Error in $\phi$", "Interpreter", "latex")
% legend("$e_{\phi}$","interpreter", "latex")
% xlabel("Iteration Number", "Interpreter","latex")
% ylabel("Orientation (deg)", "Interpreter","latex")

% figure(4)
% plot(q(1,:), q(2,:), qd(1,:), qd(2,:))
% legend("Pose", "Desired Pose")
% xlim([-0.7, 0.7])
% ylim([-0.5, 0.5])
% grid on;
% 
% figure(5)
% subplot(3,1,1)
% plot(timeVec , q(1,:), timeVec ,qd(1,:));
% title("Plot of x", "Interpreter", "latex")
% legend("x", "$x_d$","Interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(timeVec , q(2,:), timeVec ,qd(2,:));
% title("Plot of y", "Interpreter", "latex")
% legend("y", "$y_d$","Interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(timeVec , rad2deg(q(3,:)), timeVec, rad2deg(qd(3,:)) );
% title("Plot of $\phi$", "Interpreter", "latex")
% legend("$\phi$", "$\phi _d$","Interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Angle (deg)", "Interpreter","latex")
% 
% % figure(6)
% % subplot(3,1,1)
% % plot(timeVec, e(1,:));
% % title("Error in x", "Interpreter", "latex")
% % legend("$e_x$","interpreter", "latex")
% % xlabel("Time (s)", "Interpreter","latex")
% % ylabel("Position (m)", "Interpreter","latex")
% % 
% % subplot(3,1,2)
% % plot(timeVec, e(2,:));
% % title("Error in y", "Interpreter", "latex")
% % legend("$e_y$","interpreter", "latex")
% % xlabel("Time (s)", "Interpreter","latex")
% % ylabel("Position (m)", "Interpreter","latex")
% % 
% % subplot(3,1,3)
% % plot(timeVec , rad2deg(e(3,:)));
% % title("Error in $\phi$", "Interpreter", "latex")
% % legend("$e_{\phi}$","interpreter", "latex")
% % xlabel("Time (s)", "Interpreter","latex")
% % ylabel("Orientation (deg)", "Interpreter","latex")
% % 
% % 
% % figure(7)
% % subplot(3,1,1)
% % plot(timeVec, e_int(1,:));
% % title("Error in x", "Interpreter", "latex")
% % legend("$e_x$","interpreter", "latex")
% % xlabel("Time (s)", "Interpreter","latex")
% % ylabel("Position (m)", "Interpreter","latex")
% % 
% % subplot(3,1,2)
% % plot(timeVec , e_int(2,:));
% % title("Error in y", "Interpreter", "latex")
% % legend("$e_y$","interpreter", "latex")
% % xlabel("Time (s)", "Interpreter","latex")
% % ylabel("Position (m)", "Interpreter","latex")
% % 
% % subplot(3,1,3)
% % plot(timeVec ,(e_int(3,:)));
% % title("Error in $\phi$", "Interpreter", "latex")
% % legend("$e_{\phi}$","interpreter", "latex")
% % xlabel("Time (s)", "Interpreter","latex")
% % ylabel("Orientation (deg)", "Interpreter","latex")
% % 
% %  
% % figure(8)
% % subplot(2,1,1)
% % plot(q(1,:), q(2,:), qd(1,:), qd(2,:))
% % legend("Pose", "Desired Pose")
% % xlim([-0.7, 0.7])
% % ylim([-0.5, 0.5])
% % grid on;
% 
% figure(9)
% subplot(2,1,1)
% hold on
% plot(timeVec,f);
% legend('$f_1$','$f_2$','$f_3$','$f_4$',"Interpreter", "latex")
% subplot(2,1,2)
% plot(timeVec,T_friction)
% legend('$f_1$','$f_2$','$f_3$','$f_4$',"Interpreter", "latex")
% 
% figure(10)
% title("Friction vs Velocity ")
% subplot(2,2,3)
% plot(fvel(1,:), T_friction(1,:), 'o')
% title("ODrive0")
% xlabel("Velocity","Interpreter","latex")
% ylabel("Force", "Interpreter","latex")
% 
% subplot(2,2,1)
% plot(fvel(2,:), T_friction(2,:),'o')
% title("ODrive1")
% xlabel("Velocity","Interpreter","latex")
% ylabel("Force", "Interpreter","latex")
% 
% subplot(2,2,2)
% plot(fvel(3,:), T_friction(3,:),'o')
% title("ODrive2")
% xlabel("Velocity","Interpreter","latex")
% ylabel("Force", "Interpreter","latex")
% 
% subplot(2,2,4)
% plot(fvel(4,:), T_friction(4,:),'o')
% title("ODrive3")
% xlabel("Velocity","Interpreter","latex")
% ylabel("Force", "Interpreter","latex")
% 
% figure(11)
% subplot(2,2,3)
% plot(timeVec, vel(1,:)); hold on;
% plot(timeVec, fvel(1,:)); hold off;
% title("ODrive0", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,1)
% plot(timeVec, vel(2,:)); hold on;
% plot(timeVec, fvel(2,:)); hold off;
% title("ODrive1", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,2)
% plot(timeVec, vel(3,:)); hold on;
% plot(timeVec, fvel(3,:)); hold off;
% title("ODrive2", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,4)
% plot(timeVec, vel(4,:)); hold on;
% plot(timeVec, fvel(4,:)); hold off;
% title("ODrive3", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% %%
% figure(12)
% plot(timeVec,w_friction(1,:)+w_c(1,:))
% legend("$w_{fric,x}$","$w_y$","$\tau_z$","Interpreter", "latex")
% 
% figure(13)
% plot(timeVec, w_c(1,:))
% legend("$w_x$","$w_y$","$\tau_z$","Interpreter", "latex")
% %%
% cable = 1;
% figure(14)
% hold on
% plot(tIter,f(cable,:))
% plot(tIter,f_friction(cable,:))
% % plot(tIter,f_friction_signed(cable,:),'--')
% hold off
% 
% cable = 2;
% figure(15)
% hold on
% plot(timeVec,f(cable,:))
% plot(timeVec,f_friction(cable,:))
% % plot(timeVec,f_friction_signed(cable,:),'--')
hold off

