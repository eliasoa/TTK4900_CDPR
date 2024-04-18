function AT = structure_matrix(a_i, b_b, q, r_p, betar)
    x = q(1);
    y = q(2);
    theta = q(3);
    
    r = [x;y];

    beta1p = pi   + betar(1);
    beta2p = pi   - betar(2);
    beta3p = 0    + betar(3);
    beta4p = 2*pi - betar(4);

    betap = [beta1p; beta2p; beta3p; beta4p];

    c1 = [a_i(1,1) + r_p*(1 + cos(betap(1)));a_i(2,1)+r_p*sin(betap(1))];
    c2 = [a_i(1,2) + r_p*(1 + cos(betap(2)));a_i(2,2)+r_p*sin(betap(2))];
    c3 = [a_i(1,3) + r_p*(cos(betap(3)) - 1);a_i(2,3)+r_p*sin(betap(3))];
    c4 = [a_i(1,4) + r_p*(cos(betap(4)) - 1);a_i(2,4)+r_p*sin(betap(4))];
    
    ci = [c1 c2 c3 c4];
    
    R = R_z(theta);
    
    bi = [R*b_b(:,1) R*b_b(:,2) R*b_b(:,3) R*b_b(:,4)];
    l1 = ci(:,1) - r - bi(:,1);
    l2 = ci(:,2) - r - bi(:,2);
    l3 = ci(:,3) - r - bi(:,3);
    l4 = ci(:,4) - r - bi(:,4);

    u1 = l1/norm(l1,2);
    u2 = l2/norm(l2,2);
    u3 = l3/norm(l3,2);
    u4 = l4/norm(l4,2);
    u = [u1 u2 u3 u4];

    bcrossu = zeros(1,4);
    for i = 1:4
        biskew     = skew([bi(:,i); 0]);
        crossProd  = biskew*[u(:,i);0];
        bcrossu(i) = crossProd(3);
    end

    AT = [u;bcrossu];
end


function uskew = skew(u)
    u1 = u(1);
    u2 = u(2);
    u3 = u(3);
    uskew = [ 0  -u3  u2;
              u3  0  -u1;
             -u2  u1  0];
end
function R = R_z(theta)
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
end