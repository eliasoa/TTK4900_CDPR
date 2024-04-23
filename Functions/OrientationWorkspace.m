function [phi_min, phi_max] = OrientationWorkspace(r_0,a,b, f_min,f_max,f_ref, w,r_p)
% Function for calculating the Orientation Workspace of a Cable-Driven
% Parallel Robot. Inspired by the work presented by Pott in "Cable-Driven
% Parallel Robots: Theory and Application", p. 161.
%
% Inputs:
%   r_0     : The cartesian position of the mobile platform
%   a       : ... Anchor Points for cable attachment 
%   b       : Body Anchor Points for cable attachment 
%   f_min   : Minimum Limit on cable force
%   f_max   : Maximum Limit on cable force
%   f_ref   : Desired cable force 
%   w       : Desired Wrench
%   r_p     : Radius of pulleys
%
% Outputs:
%
%   phi_min : Minimum angle for the platform in r_0
%   phi_max : Maximum angle for the platform in r_0
%
% Created by Magnus Grøterud


% Grid of angles to analuze
phi = linspace(-45, 45, 360*2);
P = length(phi);

% Allocations for solutions
f_positive = zeros(P,1);

f_prev = f_ref*ones(4,1);

for i=1:P
    % Convert to radians
    phiRad = deg2rad(phi(i));

    % Define current pose to analyze
    q = [r_0;phiRad];

    [~,betar] = p_inverse_kinematics(a,b,q, r_p);
    A_t = structure_matrix(a,b,q,r_p, betar);
    A = A_t';

    % Calculate Force Allocation 
    
    [~,~, flag]  = ForceAllocIterativeSlack(A,f_min, f_max, f_ref, f_prev, w);
    if flag == 0
        f_positive(i) = 1;
    end
end


% Plot
plot(phi,f_positive)
title("Orientation Workspace")
xlabel("$\phi$ [Deg]", "Interpreter","latex")
ylabel("Boolean Scale","Interpreter","latex")
ylim([-1,2])
grid on;

% Find the index of the first occurrence of 1
firstIndex = find(f_positive == 1, 1, 'first');

% Find the index of the last occurrence of 1
lastIndex = find(f_positive == 1, 1, 'last');


% Define the interval of angles for the position
phi_min = phi(firstIndex);
phi_max = phi(lastIndex);