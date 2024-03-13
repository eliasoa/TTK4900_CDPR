function l_dot = encoder2cableVel(encoderVel, CDPR_Params, motorsign)
% Function for calculating rate of change of cable length winded up on a
% spool from velocity measurement from an encoder. Made by Magnus Grøterud
%
% l_dot         : "Cable Velocity" (Rate of change of cable length) [m/s]
% encoderVel    : Angular velocity from encoder                     [turns/s]
% CDPR_Params   : Struct with parameters for the CDPR


% Extract neccessary Params
R = CDPR_Params.Gen_Params.SPOOL_RADIUS;

% Convert to radians/second
theta_dot = encoderVel*2*pi*motorsign;

% Calculate l_dot
l_dot = theta_dot*R;





