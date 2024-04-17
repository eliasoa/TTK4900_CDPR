function l_est = encoder2cableLen(pos, l_0, R, motorsign, L0,hs)
% pos           : Current Encoder Position Estimate [rad]
% l0            : Length of the cable when encoder_pos = 0 
%                (When the platform is centered in the frame) [m]
% R             : Radius of spool/winch [m]
% motorsign     : -1 if cable points down from spool
%                  1 if cable points up from spool


% % Convert position to radians
% pos_rad = 2*pi*pos*motorsign;
% 
% % Estimated cable length
% l_est = l_0 + pos_rad*R;  

%% Calculate Total Cable Length
% Convert position to radians
pos_rad = 2*pi*pos*motorsign;

% Estimated Totatl cable length
L_tot = L0 + pos_rad*R;  

%% Calculate Cable Length between spool and pulley
% Distance between the pitches
D_c = 3*10^(-3);

% Caluclate linear position of cable outlet on the spool
x = D_c*pos;

% Calculate cable length from spool to pulley
l_s = sqrt(x^2 + h_s ^2);

% Calculate Cable length on the pulley
l_p = ;

%% Calculate Cable Length States
l_est = L_tot - l_s - l_p;

end








