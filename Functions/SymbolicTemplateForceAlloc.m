%% Symbolic Functions For Force Allocation Algorithm
syms f1 f2 f3 f4 f_min f_max f_ref c p s1 s2 s3 b epsilon 

%% Force Optimization Variable
f = [f1;f2;f3;f4];
% Calculate constants 
alpha = (f_max-f_min)/2;

c1 = c;
c2 = c;

% Compute the Objective Function
g_f = 0;
for i = 1:4
    g_i = (f(i) - f_ref)^p/alpha^p - c1*log(f(i) - f_min) - c2*log(f_max - f(i));
    g_f = g_f + g_i;
end

% Compute the gradient of the Objective Function
Gradient_f = jacobian(g_f, f);

% Compute the Hessian of the Objective Function
Hessian_f = jacobian(Gradient_f, f);

%% Slack Variable
s = [s1;s2;s3];
% Calculate Objective Function for slack Variables
g_s = 0;
for i=1:3
    g_s_i = b*sqrt(epsilon + s(i)^2) + s(i)^2;
    g_s = g_s + g_s_i;
end

% Calculate Gradient of objective function
GradG_S = jacobian(g_s, s);

% Calculate Hessian of objective function
HessianG_s = jacobian(GradG_S, s);

%% Test Both
x_sym = [f;s];

% Total Objective Function
g_x = g_f + g_s;

% Total Gradient
Gradient_x = jacobian(g_x, x_sym);

% Total Hessian
Hessian_x = jacobian(Gradient_x, x_sym);


%% Numerical functions

matlabFunction(g_x, 'vars', {x_sym, f_min, f_max, f_ref, p, c b, epsilon}, "File", "ObjectiveFun");
matlabFunction(Gradient_x, 'vars', {x_sym, f_min, f_max, f_ref, p, c b, epsilon}, "File", "GradientObj");
matlabFunction(Hessian_x, 'vars', {x_sym, f_min, f_max, f_ref, p, c b, epsilon},"File", "HessianObj");

