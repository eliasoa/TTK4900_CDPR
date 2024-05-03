
Data = load("PlotData/PoseData.mat");

q           = Data.q_log;
qd          = Data.qd_log;
e           = Data.e_log;  
e_int       = Data.e_int_log;  
timeVec     = Data.timeVec;

iterations    = length(q);
tIter = linspace(1,iterations,iterations);

%% Plot of pose and desired pose

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
% 
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
% 
% 
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


figure(3)
plot(q(1,:), q(2,:), qd(1,:), qd(2,:))
legend("Pose", "Desired Pose")
xlim([-0.7, 0.7])
ylim([-0.5, 0.5])
grid on;


figure(1)
subplot(3,1,1)
plot(timeVec , q(1,:), timeVec ,qd(1,:));
title("Plot of x", "Interpreter", "latex")
legend("x", "$x_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,2)
plot(timeVec , q(2,:), timeVec ,qd(2,:));
title("Plot of y", "Interpreter", "latex")
legend("y", "$y_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,3)
plot(timeVec , rad2deg(q(3,:)), timeVec, rad2deg(qd(3,:)) );
title("Plot of $\phi$", "Interpreter", "latex")
legend("$\phi$", "$\phi _d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Angle (deg)", "Interpreter","latex")

% figure(2)
% subplot(3,1,1)
% plot(timeVec, e(1,:));
% title("Error in x", "Interpreter", "latex")
% legend("$e_x$","interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(timeVec, e(2,:));
% title("Error in y", "Interpreter", "latex")
% legend("$e_y$","interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(timeVec , rad2deg(e(3,:)));
% title("Error in $\phi$", "Interpreter", "latex")
% legend("$e_{\phi}$","interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Orientation (deg)", "Interpreter","latex")
% 
% 
% figure(3)
% subplot(3,1,1)
% plot(timeVec, e_int(1,:));
% title("Error in x", "Interpreter", "latex")
% legend("$e_x$","interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(timeVec , e_int(2,:));
% title("Error in y", "Interpreter", "latex")
% legend("$e_y$","interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(timeVec ,(e_int(3,:)));
% title("Error in $\phi$", "Interpreter", "latex")
% legend("$e_{\phi}$","interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Orientation (deg)", "Interpreter","latex")
% 
% %  
% % figure(3)
% % subplot(2,1,1)
% % plot(q(1,:), q(2,:), qd(1,:), qd(2,:))
% % legend("Pose", "Desired Pose")
% % xlim([-0.7, 0.7])
% % ylim([-0.5, 0.5])
% % grid on;



