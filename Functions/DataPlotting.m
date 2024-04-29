
% Load Data file(s)
Data = load("PoseData.mat");

% Extract Data
q   = Data.q_log;
qd  = Data.qd_log;
e   = Data.e_log;

iterMax = length(q);
iter = linspace(0,iterMax,iterMax+1);

%% Step Response of Pose
figure(2)
subplot(3,1,1)
hold on
plot(iter, q(1,:),"linestyle","-","Color","red")
plot(iter, qd(1,:),"linestyle","--","Color","red")
hold off
title("Trajectory of x", "Interpreter","latex")
legend("$x$","$x_d$", "interpreter", "latex")
xlabel("Iteration Number", "Interpreter","latex")
ylabel("Position [m]", "Interpreter","latex")

subplot(3,1,2)
hold on
plot(iter, q(2,:),"linestyle","-","Color","blue")
plot(iter, qd(2,:),"linestyle","--","Color","blue")
hold off
title("Trajectory of y", "Interpreter","latex")
legend("$y$","$y_d$", "interpreter", "latex")
xlabel("Iteration Number", "Interpreter","latex")
ylabel("Position [m]", "Interpreter","latex")

subplot(3,1,3)
hold on
plot(iter, q(3,:),"linestyle","-","Color","green")
plot(iter, qd(3,:),"linestyle","--","Color","green")
hold off
title("Trajectory of x", "Interpreter","latex")
legend("$\phi$","$\phi_d$", "interpreter", "latex")
xlabel("Iteration Number", "Interpreter","latex")
ylabel("Pose [rad]", "Interpreter","latex")