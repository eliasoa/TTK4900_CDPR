

%% Initialization

% Preamble
clc
clear all

% Params
L       = 0.4;
omega   = 8;
h       = 0.1;

% w_ref = [0;0;0];

init_CDPR_Params

a = CDPR_SGM.FrameAP;
b = CDPR_SGM.BodyAP.RECTANGLE;

f_min = 5;
f_max = 60;
f_ref = 45;

%% Memory Allocation
N       = 200;
f       = zeros(4,N);

q0 = [0;0;0];
qd0 = [0;0;0];

q       = zeros(3,N);
q(:,1)  = q0;

q_dot   = zeros(3,N);
q_ddot  = zeros(3,N);

qd      = zeros(3,N);
qd(:,1) = qd0;
f_prev  = f_ref*ones(4,1);
w_des   = zeros(3,N);
w_res  = zeros(3,N);

%% Control
Kp  = 20;
Kd  = 10;

m   = 0.01;
Iz  = 2.06250*10^(-7);
M = diag([0.1 0.1 0.1]);
% M_inv = inv(M)
M_inv = diag([10 10 10]);


%% Test Loop
for i =1:N
    
    A = WrenchMatrix_V2(a,b,q(:,i));

    % e = qd(:,i) - q(:,i);
    e = qd(:,i) - q(:,i);
    w_des(:,i) = Kp*e; + Kd*q_dot(:,i);

    [f(:,i), w_res(:,i)] = ForceAllocIterativeSlack(A,f_min,f_max,f_ref,f_prev,w_des(:,i));
    
    % Update f_prev
    f_prev = f(:,i);

    % Update q
    t = (i-1)*h;
    x = L*sin(omega*t);
    y = L*cos(omega*t);
    qd(:,i+1) = [x;y;0];

    % % Acceleration
    q_ddot(:,i+1)   = M_inv*(A'*f(:,i));
    q_dot(:,i+1)    = q_dot(:,i) + q_ddot(:,i)*h;
    % q(:,i+1)        = q(:,i) + q_dot(:,i)*h;
end

%% Plotting
h = 0.01;
t = 0:h:(N-1)*h;

figure(1)

subplot(3,1,1)
plot([t N*h], qd,'--');hold on
plot([t], q);hold off
legend("x_d","y_d","phi_d","x","y","phi")
xlabel("t")
ylabel("Position nice")
grid on

subplot(3,1,2)
plot([t N*h], q_dot)
legend("$\dot{x}$","$\dot{y}$","$\dot{\phi}$",'Interpreter','latex');
xlabel("t")
ylabel("Velocity nice")
grid on

subplot(3,1,3)
plot([t N*h], q_ddot)
legend("$\ddot{x}$","$\ddot{y}$","$\ddot{\phi}$",'Interpreter','latex');
xlabel("t")
ylabel("Acceleration nice")
grid on

figure(2)


subplot(2,1,1)
plot(t,f)
title("Calculated Cable Forces, \textbf{f}",'Interpreter','latex')
legend("$f_1$","$f_2$","$f_3$","$f_4$",'interpreter','latex')
xlabel("t")
ylabel("Newton")
 ylim([20,60])
grid on

subplot(2,1,2)
plot(t,w_des,t,w_res)
title("Wrench Tracking \textbf{w}",'Interpreter','latex')
legend("$F_{x,desired}$","$F_{y,desired}$","$M_{\phi ,desired}$","$F_{x,res}$","$F_{y,res}$","$M_{\phi ,res}$",'interpreter','latex')
xlabel("t")
ylabel("Newton/Newton meter")
grid on




