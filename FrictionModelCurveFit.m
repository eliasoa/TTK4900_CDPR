
%% Curve fit polynomial

% Physical parameters
m               = 5;                    % mass of weight[kg]
r_d             = 20*1e-3;              % drum radius [m]
g               = 9.81;                 % acc due to grav [m/s^2]
t_theoretical   = m*g*r_d*ones(4,1);    % Theoretical torque required [Nm]


% Velocities lifting test performed at
v = [0.5,    1,     2.5,   4]';         % Velocities [rev/s]
% Initial points for lsqnonlin, value seems to no matter
x0      = [0.5,0.5,0.5];
x0_2    = [0.5,0.5,0.5,0.5];
% Points to intersect b curve to pos and neg curve
v2 = [-0.5, 0, 0.5];

%% ODrive0
% Loss in torque = easured torqes aquired by experiments - theoretical torques needed
t0 = [1.039, 1.039, 1.042, 1.0505]' - t_theoretical;
% Convert to force
% t0 = tau0/r_d;

% Function for eq(4)
fun = @(c)c(1).*v.^2 + c(2)*v + c(3) - t0;

% Function for eq(5) v in [-v_L ; v_L], attempting to ensure that the
% function t is continious
fun2 = @(b)b(1)*v2+b(2)*v2.^2 + b(3)*v2.^3 + b(4)*v2.^4  - [-t0(1) 0 t0(1)];

c0 = lsqnonlin(fun,x0);

b0 = lsqnonlin(fun2,x0_2, [], [], [],[],[],[], @(b)nlcon(b, c0, v2));
% Add b0 = 0
b0 = [0,b0];
%% ODrive1
t1 = [1.04, 1.042, 1.043, 1.051]' - t_theoretical;
% t1 = tau1/r_d;

% Function for eq(4)
fun = @(c)c(1).*v.^2 + c(2)*v + c(3) - t1;

% Function for eq(5) v in [-v_L ; v_L], attempting to ensure that the
% function t is continious
fun2 = @(b)b(1)*v2+b(2)*v2.^2 + b(3)*v2.^3 + b(4)*v2.^4  - [-t1(1) 0 t1(1)];

c1 = lsqnonlin(fun,x0);

b1 = lsqnonlin(fun2,x0_2, [], [], [],[],[],[], @(b)nlcon(b, c1, v2));
% Add b0 = 0
b1 = [0,b1];
%% ODrive2
t2 = [1.03, 1.0305, 1.037, 1.042]' - t_theoretical;
% t2 = tau2/r_d;

% Function for eq(4)
fun = @(c)c(1).*v.^2 + c(2)*v + c(3) - t2;

% Function for eq(5) v in [-v_L ; v_L], attempting to ensure that the
% function t is continious
fun2 = @(b)b(1)*v2+b(2)*v2.^2 + b(3)*v2.^3 + b(4)*v2.^4  - [-t2(1) 0 t2(1)];


c2 = lsqnonlin(fun,x0);

b2 = lsqnonlin(fun2,x0_2, [], [], [],[],[],[], @(b)nlcon(b, c2, v2));
% Add b0 = 0
b2 = [0,b2];

%% ODrive3
t3 = [1.028, 1.03, 1.039, 1.043]' - t_theoretical;
% t3 = tau3/r_d;

% Function for eq(4)
fun = @(c)c(1).*v.^2 + c(2)*v + c(3) - t3;

% Function for eq(5) v in [-v_L ; v_L], attempting to ensure that the
% function t is continious
fun2 = @(b)b(1)*v2 + b(2)*v2.^2 + b(3)*v2.^3 + b(4)*v2.^4 - [-t3(1) 0 t3(1)];


c3 = lsqnonlin(fun,x0);
options = optimoptions('lsqnonlin','FunctionTolerance',1e-20)
b3 = lsqnonlin(fun2,x0_2, [], [], [],[],[],[], @(b)nlcon(b, c3, v2),options);
b3 = [0, b3];


%% Plot

% Velocity x axis
b_x = linspace(-v(1), v(1));
c_x = linspace(v(1),v(4));

b_0_func = b0(1) + b0(2)*b_x.^1 + b0(3)*b_x.^2 + b0(4)*b_x.^3;
b_1_func = b1(1) + b1(2)*b_x.^1 + b1(3)*b_x.^2 + b1(4)*b_x.^3;
b_2_func = b2(1) + b2(2)*b_x.^1 + b2(3)*b_x.^2 + b2(4)*b_x.^3;
b_3_func = b3(1) + b3(2)*b_x.^1 + b3(3)*b_x.^2 + b3(4)*b_x.^3;

c_0_func = c0(1)*c_x.^2 + c0(2)*c_x + c0(3);
c_1_func = c1(1)*c_x.^2 + c1(2)*c_x + c1(3);
c_2_func = c2(1)*c_x.^2 + c2(2)*c_x + c2(3);
c_3_func = c3(1)*c_x.^2 + c3(2)*c_x + c3(3);



figure(1)
hold on;
plot(b_x, b_0_func); 
plot(c_x, c_0_func);
plot(-c_x, -c_0_func);
hold off;

figure(2)
hold on;
plot(b_x, b_1_func); 
plot(c_x, c_1_func);
plot(-c_x, -c_1_func);
hold off;

figure(3)
hold on;
plot(b_x, b_2_func); 
plot(c_x, c_2_func);
plot(-c_x, -c_2_func);hold off;

figure(4)
hold on;
plot(b_x, b_3_func); 
plot(c_x, c_3_func);
plot(-c_x, -c_3_func);
hold off;
%% Functions
function [c,ceq] = nlcon(b, c, v)
% Equality constraints
ceq = [b(1)*v(3) + b(2)*v(3).^2 + b(3)*v(3).^3 + b(4)*v(3).^4 - (c(1)*v(3).^2 + c(2)*v(3) + c(3));
       b(1)*v(1) + b(2)*v(1).^2 + b(3)*v(1).^3 + b(4)*v(1).^4 + (c(1)*v(3).^2 + c(2)*v(3) + c(3))
       b(1) + 2*b(2)*v(3) + 3*b(3)*v(3).^2 + 4*b(4)*v(3).^3 - (2*c(1)*v(3) + c(2));
       b(1) + 2*b(2)*v(1) + 3*b(3)*v(1).^2 + 4*b(4)*v(1).^3 - (2*c(1)*v(1) + c(2))];
c = [];
end