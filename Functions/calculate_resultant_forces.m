function wr = calculate_resultant_forces(cable_forces, b, q)
theta = q(3);
R = R_z(theta);
m = zeros(1,4);
for i = 1:4
    cross_prod = cross([R*b(:,i);0],[cable_forces(:,i);0]);
    m(i) = cross_prod(3);
end

f_res = sum(cable_forces,2);
m_res = sum(m, 2);
wr = [f_res;m_res];
end

% Local Functions
function R = R_z(theta)
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
end