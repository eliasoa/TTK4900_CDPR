function [l,betar] = p_inverse_kinematics(a, b_b, pose, r_p)
    x = pose(1);
    y = pose(2);
    theta = pose(3);
    r = [x;y];

    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
    
    R1 = [1 0;
          0 cos(pi)];
    
    R2 = [1 0;
          0 1];
    
    R3 = [cos(pi) 0;
          0       cos(0)];
    
    R4 = [cos(pi) 0;
          0       cos(pi)];

    b1p = R1*(R*b_b(:,1) + r - a(:,1));
    b2p = R2*(R*b_b(:,2) + r - a(:,2));
    b3p = R3*(R*b_b(:,3) + r - a(:,3));
    b4p = R4*(R*b_b(:,4) + r - a(:,4));
    
    bp = [b1p b2p b3p b4p];
    
    lf1 = sqrt(bp(1,1)^2 - 2*bp(1,1)*r_p + bp(2,1)^2);
    lf2 = sqrt(bp(1,2)^2 - 2*bp(1,2)*r_p + bp(2,2)^2);
    lf3 = sqrt(bp(1,3)^2 - 2*bp(1,3)*r_p + bp(2,3)^2);
    lf4 = sqrt(bp(1,4)^2 - 2*bp(1,4)*r_p + bp(2,4)^2);
    l = [lf1; lf2; lf3;lf4];

    mb1 = sqrt((bp(1,1)-r_p)^2 + bp(2,1)^2);
    mb2 = sqrt((bp(1,2)-r_p)^2 + bp(2,2)^2);
    mb3 = sqrt((bp(1,3)-r_p)^2 + bp(2,3)^2);
    mb4 = sqrt((bp(1,4)-r_p)^2 + bp(2,4)^2);
    
    % beta_1
    alpha1 = acos(lf1/mb1);
    alpha2 = acos(lf2/mb2);
    alpha3 = acos(lf3/mb3);
    alpha4 = acos(lf4/mb4);
    
    % beta_2
    gamma1 = acos(bp(2,1)/mb1);
    gamma2 = acos(bp(2,2)/mb2);
    gamma3 = acos(bp(2,3)/mb3);
    gamma4 = acos(bp(2,4)/mb4);
    
    % Pulley cable entry angle
    beta_e = 0.17896;
    beta_e = 0.0;
    
    beta1r = alpha1 + gamma1 - beta_e;
    beta2r = alpha2 + gamma2 - beta_e;
    beta3r = alpha3 + gamma3 - beta_e;
    beta4r = alpha4 + gamma4 - beta_e;
    betar = [beta1r; beta2r; beta3r; beta4r];
end