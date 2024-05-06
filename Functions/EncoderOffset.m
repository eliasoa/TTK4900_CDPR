function pos = EncoderOffset(pos, p0)
% pos = EncoderOffset(pos, p0)
% Function that calculates and handles the offset from the encoders true
% zero to the zero we want xdddd
% p0 = the encoder positions when the MP is at q = [0 0 0]'
% pos = encoder reading
p1 = pos(1);
p2 = pos(2);
p3 = pos(3);
p4 = pos(4);

% p1 = sign(p1)*(abs(p1)-abs(p0(1)));
% p2 = sign(p2)*(abs(p2)-abs(p0(2)));
% p3 = sign(p3)*(abs(p3)-abs(p0(3)));
% p4 = sign(p4)*(abs(p4)-abs(p0(4)));

p1 = p1 + p0(1)*(-1);
p2 = p2 + p0(2)*(-1);
p3 = p3 + p0(3)*(-1);
p4 = p4 + p0(4)*(-1);
% if p1 >= 0
%     p1 = p1 - p0(1);
% else
%     p1 = p1 + abs(p0(1));
% end
% 
% if p2 >= 0
%     p2 = p2 - p0(2);
% else
%     p2 = p0(2) - p2;
% end
% 
% if p3 >= 0
%     p3 = p3 - p0(3);
% else
%     p3 = p0(3) - p3;
% end
% 
% if p4 >= 0
%     p4 = p4 - p0(4);
% else
%     p4 = p0(4) - p4;
% end


pos = [p1;p2;p3;p4];


end