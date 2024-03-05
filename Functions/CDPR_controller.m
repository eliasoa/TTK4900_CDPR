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

K_r = controllParams(1);
K_f = controllParams(2);
K_a = controllParams(3);
K_d = controllParams(4);
d_c = controllParams(5);

% Calculate desired wrench
w_d = MotionController(s,s_d,K_r, K_f, K_a, K_d, d_c);

% Calculate Optimal Force Distribution, to generate the desired wrench
[f, flag] = Optimal_ForceDistributions(A,w_d,m_p,f_min,f_max, f_ref);

%%%%%%%%%%%%%%%%%%% Error Handling? %%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate motor torques
T_friction  = ??;
eta         = ??;
T_desired   = R*F - T_friction/eta; 

% Return the desired torques
t1 = T_desired(1);
t2 = T_desired(2);
t3 = T_desired(3);
t4 = T_desired(4);
end