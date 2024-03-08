function [t1,t2,t3,t4, f] = CDPR_controller(s, s_d, CDPR_Params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [t1,t2,t3,t4] : Commanded/desired motor torques
% s_d           : 6x1 Vector of Desired states (xd;yd;phid;speeds)
%
%
% encoder_zeros : 4x1 Vector of defined origins of the encoder 
% l0            : 4x1 Vector of initial cable lengths (based on origin of encoder)
% R             : Scalar Radius of spools
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



K_r = CDPR_Params.ControlParams.K_r;
K_f = CDPR_Params.ControlParams.K_f;
K_a = CDPR_Params.ControlParams.K_a;
K_d = CDPR_Params.ControlParams.K_d;
d_c = CDPR_Params.ControlParams.d_c;

% HENTE CONTROL PARAMS FRA STRUCTEN

% Calculate desired wrench
w_d = K_r*s_d-[K_f; K_a]*s - K_d*d_c;

% Robot parameters
A = ...
m_p = CDPR_Params.Gen_Params.Platform_mass;
f_min = 1;
f_max = 50;
f_ref = 25;

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