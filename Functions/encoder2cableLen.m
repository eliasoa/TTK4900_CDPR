function l_est = encoder2cableLen(pos, l_0, R, motorsign)
% pos           : Current Encoder Position Estimate [Turns]
% l0            : Length of the cable when encoder_pos = 0 
%                (When the platform is centered in the frame) [m]
% R             : Radius of spool/winch [m]
% motorsign     : -1 if cable points down from spool
%                 1 if cable points up from spool


% Convert position to radians
pos_rad = 2*pi*pos*motorsign;

% Estimated cable length
l_est = l_0 + pos_rad*R;  

end



