function l_est = encoder2cableLen(pos0, pos, l_0, R, motorsign)
% pos0          : Initial Encoder Position Estimate [Turns]
% pos           : Current Encoder Position Estimate [Turns]
% l0            : Length of the cable when encoder_pos = 0 
%                (When the platform is centered in the frame) [m]
% R             : Radius of spool/winch [m]
% motorsign     : 0 if cable points down from spool
%                 1 if cable points up from spool

% Relative Position (OBS: SIGN MAYBE OPPOSITE, NEED TO TEST ON ACTUAL ROBOT)
if motorsign
    p_rel = pos - pos0;
else
    p_rel = pos0 - pos;
end

% Convert position to radians
pos_rad = 2*pi*p_rel;

% Estimated cable length
l_est = l_0 + pos_rad*R;  

end



