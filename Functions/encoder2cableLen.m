function l_est = encoder2cableLen(encoder_pos, l_0, R)
% encoder_pos   : Position relative to a defined 0  [Turns]
% l0            : Length of the cable when encoder_pos = 0 
%                (When the platform is centered in the frame) [m]
% R             : Radius of spool/winch [m]

% Convert position to radians
pos_rad = 2*pi*encoder_pos;


l_est = l_0 + pos_rad*R;  % ???

end



