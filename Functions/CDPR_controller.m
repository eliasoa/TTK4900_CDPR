function [t1,t2,t3,t4] = CDPR_controller(q, q_d, controllParams)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [t1,t2,t3,t4] : Commanded/desired motor torques
% q_d           : 3x1 Vector of Desired states (xd;yd;phid)
%
%
% encoder_zeros : 4x1 Vector of defined origins of the encoder 
% l0            : 4x1 Vector of initial cable lengths (based on origin of encoder)
% R             : Scalar Radius of spools
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define full states
s   = [q;0;0;0];
s_d = [q_d;0;0;0];

% K_r = controllParams(1);
% K_f = controllParams(2);
% K_a = controllParams(3);
% K_d = controllParams(4);
% d_c = controllParams(5);
mp  = 0.1;     % Mass of platform
Izz = 6.25e-4;  % Moment of inertia about z-axis
dtx = 0.0;      % Translational dampening coefficient in the x-direction
dty = 0.0;      % Translational dampening coefficient in the y-direction
dr  = 0.0;      % Rotational dampening coefficient about the z-axis
g   = 9.81;     % Gravitational acceleration

A_c = [0  0 0  1       0       0;        % 
       0  0 0  0       1       0;
       0  0 0  0       0       1;
       0  0 0 -dtx/mp  0       0;
       0  0 0  0      -dty/mp  0;
       0  0 0  0       0      -dr/Izz];

B_c = [0    0    0;
       0    0    0;
       0    0    0;
       1/mp 0    0;
       0    1/mp 0;
       0    0    1/Izz;];
d_c = [ 0;
        0;
        0;
        0;
       -g;
        0];

K_d = B_c\eye(6);
K_f = [180*diag([1,1,1]) 10*diag([1,1,1])];
K_r = pinv((B_c*K_f-A_c)\B_c);
K_a = [diag([-10,-10,-1]) zeros(3,3)];

% Calculate desired wrench
w_d = MotionController(s,s_d,K_r, K_f, K_a, K_d, d_c);

% Calculate Optimal Force Distribution, to generate the desired wrench
[f, ~] = Optimal_ForceDistributions(A,w_d,m_p,f_min,f_max, f_ref); 
% TODO: Skal flagget brukes til noe 

%%%%%%%%%%%%%%%%%%% Error Handling? %%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate motor torques
T_friction  = 0;    % Sett inn riktig tall
eta         = 1;    % Sett inn riktig tall
T_desired   = R*f - T_friction/eta; 

% Return the desired torques
t1 = T_desired(1);
t2 = T_desired(2);
t3 = T_desired(3);
t4 = T_desired(4);
end