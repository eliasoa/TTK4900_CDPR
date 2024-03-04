function w_d = MotionController(s,s_d,K_r, K_f, K_a, K_d, d_c)
% Controller for calculating the desired wrench applied on the mobile
% platform.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% w_d   : Desired Wrench on the platform 
% s     : State Vector, [q;q_dot]
% s_d   : Desired State, [q_d;q_d_dot]
% K_r   :
% K_f   : 
% K_a   :
% K_d   :
% d_c   :
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Created by Sølve Nørsterud

% Calculate desired wrench
w_d = K_r*s_d-[K_f K_a]*s - K_d*d_c;