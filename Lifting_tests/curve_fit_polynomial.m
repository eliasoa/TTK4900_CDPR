% clc
% 
% rng default % for reproducibility
% d = linspace(0,3);
% 
% v = [0.5,   1,     2.5,   4];
% t = [1.026-0.981, 1.032-0.981, 1.044-0.981, 1.05-0.981];
% % y = exp(-1.3*v) + 0.05*randn(size(v));
% % tau_f = 
% % 0.5*v.^2
% fun = @(c)c(1).*v.^2 + c(2)*v + c(3) - t
% size(fun)
% z0 = [0.5,0.5,0.5];
% x = lsqnonlin(fun,z0)
% % 0.2040*5
% plot(v,t,'ko'); hold on;
% % plot(0.5*v + 0.5*v.^2 + 0.5 - t)
% plot(v, x(1)*v.^2 + x(2)*v + x(3)); %hold on;
% % plot(v,v*x(1)+x(3))



%% ODrive2
m = 5; % [kg]
r_d = 20*1e-3; % drum radius [m]
g = 9.81;
t_theoretical = m*g*r_d*ones(4,1); % Theoretical torque required [Nm]



v = [0.5,    1,     2.5,   4]'; % Velocities [rev/s]
% t = [1.0345, 1.037, 1.045, 1.052]' - t_theoretical; % Measured torques [Nm]
t = [1.03, 1.0305, 1.037, 1.042]' - t_theoretical; % Measured torques [Nm]
t./t_theoretical*100
t = t/r_d; % Convert to forces
fun = @(c)c(1).*v.^2 + c(2)*v + c(3) - t;
z0 = [0.5,0.5,0.5];
x = lsqnonlin(fun,z0)

figure(1)
d = linspace(0.5,v(4));
plot(v,t,'ko'); hold on;
plot(v, x(1)*v.^2 + x(2)*v + x(3));
plot(d, x(1)*d.^2 + x(2)*d + x(3)); hold off;

v2 = [-0.5, 0, 0.5];
-(x(1)*v(1).^2 + x(2)*v(1) + x(3))

fun2 = @(b)b(1)*v2+b(2)*v2.^2+b(3)*v2.^3 - [-t(1) 0 t(1)];
z0 = [0.5;0.5;0.5];
x2 = lsqnonlin(fun2,z0, [], [], [],[],[],[], @(b)nlcon(b, x, v2))



figure(2)
tmp = linspace(-0.5, 0.5);
plot(tmp, x2(1)*tmp + x2(2)*tmp.^2 + x2(3)*tmp.^3)

vl = v(1);
% tauvn  = -(x(1)*vl^2 + x(2)*vl + vl);
tauvn  = -t(1);
dtauvn = -(2*x(1)*vl + x(2));

tauvp = -tauvn;
dtauvp = -dtauvn;

% b0 = 0;
% b1 = -(3*(tauvn - tauvp) + vl*(dtauvn + dtauvp))/(4*vl)
% b2 =  (4*(tauvn+tauvp) + vl*(dtauvn - dtauvp))/(4*vl^2)
% b3 =  (tauvn - tauvp + vl*(dtauvn + dtauvp))/(4*vl^3)
% b4 = -(2*(tauvn + tauvp) + vl*(dtauvn - dtauvp))/(4*vl^4)
b0 = 0;
b1 = x2(1)
b2 = x2(2)
b3 = x2(3)
p0v = linspace(-vl, vl);
p0 = b0 + b1*p0v.^1 + b2*p0v.^2 + b3*p0v.^3; % + b4*p0v.^4;

figure(3)
plot(p0v, p0); hold on;
plot(d, x(1)*d.^2 + x(2)*d + x(3));
plot(-d, -(x(1)*d.^2 + x(2)*d + x(3))); hold off;

function [c,ceq] = nlcon(b, c, v)
ceq = [b(1)*v(3)+b(2)*v(3).^2+b(3)*v(3).^3 - (c(1)*v(3).^2 + c(2)*v(3) + c(3));
       b(1)*v(1)+b(2)*v(1).^2+b(3)*v(1).^3 + (c(1)*v(3).^2 + c(2)*v(3) + c(3))
       b(1)+2*b(2)*v(3)+3*b(3)*v(3).^2 - (2*c(1)*v(3) + c(2));
       b(1)+2*b(2)*v(1)+3*b(3)*v(1).^2 - (2*c(1)*v(1) + c(2))];

c = [];

end