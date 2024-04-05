function [y] = DirectKinematics_V2(a,b,l)
% [y] = DirectKinematics(a,b,l) is implementation of Algorithm 3.16 in 
% METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS 2nd Edition, April 2004 by
% K. Madsen, H.B. Nielsen, O. Tingleff
%
% It is specifically written to solve the direct kinematics of an
% overconstraind Cable Driven Paralell Robot
%
% Inputs:
% a: vector of proximal attachment points in {i} on the form
%    a   =   | a1_x ... am_x |
%            | a1_y ... am_y |
%
% b: vector of distal attachment points in {b} on the  form 
%    b   =   | b1_x ... bm_x |
%            | b1_y ... bm_y |
%
% l: length of cable i = 1,...,m
%    
%    l   =   | l1 ... lm|
%
% Outputs:
% y: vector with pose of the platform in {i} containg the carthesian
%    position vector r and the angle phi denoting the roation about the 
%    z axis [rad]
%
%    y   =   |  r  | =  |  x  | 
%            | phi |    |  y  | 
%                       | phi | 
%
%
% Author: Elias Olsen Almenningen
% Date: 16.10.2023
% Revisions:

r0          = InitialPose(l,a,b);   % Calculate inintal pose
phi0        = 0;                    % Initial estimate of phi
y           = [r0;phi0];            % Initial pose
J           = Jacobian(y,a,b,l);    % Calculate Jacobian of inital pose

epsilon1    = 10e-10; %10e-17;               % Threshold parameter 1, set by user
epsilon2    = epsilon1;             % Threshold parameter 2, set by user


A           = J.'*J;                % Firs A is without + mu*I
g           = J'*Phi(a,y,b,l);      % 

tau         = 1e-6;                 % Chosen after footnote 3
mu          = tau*max(diag(A));     % Damping factor eq (3.14)
nu          = 2;                    % Factor preventing fluctutations in mu

iter_max    = 200;                  % Max number of iterations
iter        = 0;                    % Iteration number

%cond1       = false;               % norm_2( h_i ) < e_2 * (norm_2(y_i) + e2)
%cond2       = false;               % norm_2( J(y_i) * phi(y_i) ) < e1

stop        = false;
while ~stop && iter < iter_max
    iter = iter + 1;
    h = A\-g;
    cond1 = norm(h,2) < epsilon2 * (norm(y,2));
    if cond1                            % If small step size, stop
        stop = true;
        disp("Condition 1 is true after ");
        disp(iter); 
        disp("iterations");
    else                                % Take step
        y_new = y + h;
        F_new = norm(Phi(a,y_new,b,l))^2;
        F = norm(Phi(a,y,b,l))^2;
        rho = ( F - F_new ) / ( 1/2 * h' * (mu * h - g) );
        if rho > 0
            y = y_new;
            J = Jacobian(y,a,b,l);
            A = J' * J + mu * eye(3);
            g = J' * Phi(a,y,b,l);
            cond2 = norm(g,2) < epsilon1; 
            if cond2
                stop = true;
                disp("Condition 2 is true after ");
                disp(iter); 
                disp("iterations");
            end
            mu = mu * max(1/3,1-(2*rho-1)^3);
            nu = 2;
        else
            mu = mu * nu;
            nu = 2 * nu;
        end
    end
end
if iter == iter_max
    disp("Too many iterations");
end
end


% Local Functions
function r0 = InitialPose(l,a,b)
% r0 = InitialPose(l,a,b) calculates a initial pose for the 
% tool centrepoint (TCP) of a Cable-Driven Paralell robot
% using eq. (4.40) - (4.41) in Pott, p 143
%
%
%
%
% Author: Magnus Grøterud
% Date: Kven kveit
% Revision:

% Memory Allocation
r_low = zeros(2,4);
r_high = zeros(2,4);


m = 4;

for i = 1:m
    r_low(:,i) = a(:,i) - (l(i) + norm(b(:,i))*[1;1]);
    r_high(:,i) = a(:,i) + (l(i) + norm(b(:,i))*[1;1]);
end

% Intersection of all m bounding boxes

% Cursed
r_lowMax = max(r_low);
r_highMin = min(r_high);

[~,I_Max] = max(r_lowMax);
[~,I_min] = min(r_highMin);

r0 = (r_low(:,I_Max) + r_high(:,I_min))*(0.5);

end

function J = Jacobian(y,a,b,l)
% J = Jacobian(y,a,b) calculates the Jacobian using the gradient of g
% Eq 4.43 and 4.47 in Cable-Driven Parallel Robots by Andreas Pott
% Author:    Elias Olsen Almenningen
% Date:      11 Oct 2023
% Revisions: 

[~, Gx, Gy, Gphi] = constraintGradient(y,a,b,l);

J = [Gx(1) Gy(1) Gphi(1);
     Gx(2) Gy(2) Gphi(2);
     Gx(3) Gy(3) Gphi(3);
     Gx(4) Gy(4) Gphi(4)];

end

function phi = Phi(a,y,b,l)
% phi = Phi(a,y,b,l) calculates the objective function phi(l,r,R) 
% eq. (4.46) in Pott, p 146
%
%   a   =   | a1_x ... am_x |
%           | a1_y ... am:y |
%
%   b   =   | b1_x ... bm_x |
%           | b1_y ... bm:y |
%
%   l   =   | l1 ... lm|
%
%   y   =   |  x  |
%           |  y  |
%           | phi |
%
% Author:    Elias Olsen Almenningen
% Date:      16.10.2023
% Revisions: 

% Preallocate memory
phi = zeros(4,1);    

% Calculate g = nu_i^2  eq. (4.57) in Pott p.148
for i = 1:4
    phi(i) = ( (a(1,i) - y(1) - b(1,i)*cos(y(3)) + b(2,i)*sin(y(3)))^2 ...
        +      (y(2) - a(2,i) + b(2,i)*cos(y(3)) + b(1,i)*sin(y(3)))^2 ...
        -       l(i)^2 )^2;
end
end

function [G, Gx, Gy, Gphi] = constraintGradient(y,a,b,l)
% G = gradient(y,a,b) calculates the gradient of the objective function
% g(l,r,R) = sum_i^m nu^2_i (eq. (4.46) in Pott, p 146) for 4 cables
% where a is the row vector containg the proximal anchor points
% and b is the row vector containg the distal anchor points and l is the
% row vector containg the cable lengths
%
%   a   =   | a1_x ... am_x |
%           | a1_y ... am:y |
%
%   b   =   | b1_x ... bm_x |
%           | b1_y ... bm:y |
%
%   l   =   | l1 ... lm|
%
%   y   =   |  x  |
%           |  y  |
%           | phi |
%
% Author:    Elias Olsen Almenningen
% Date:      08 Oct 2023
% Revisions: 16.10.2023 Fixed error in Gx, from b(2,i)*sin(phi) to
%                       b(1,i)*sin(phi)
m = 4;
r = y(1:2);
phi = y(3);
Gx = [0 0 0 0]';    % d/dx ( nu_1^2 ) ... d/dx ( nu_4^2 )
Gy = [0 0 0 0]';    % d/dy ( nu_1^2 ) ... d/dy ( nu_4^2 )
Gphi = [0 0 0 0]';  % d/dphi( nu_1^2 ) ... d/dphi ( nu_4^2 )
for i = 1:m
    Gx(i) = -2 * ( (a(1,i) - r(1) - b(1,i) * cos(phi) + b(2,i) * sin(phi) )^2 ...
               + (  r(2) - a(2,i) + b(2,i) * cos(phi) + b(1,i) * sin(phi) )^2 - l(i)^2)...
             * 2 *( a(1,i) - r(1) - b(1,i) * cos(phi) + b(2,i) * sin(phi));
    Gy(i) =  2 * ((a(1,i) - r(1) - b(1,i)*cos(phi) + b(2,i)*sin(phi))^2 ...
                + (r(2) - a(2,i) + b(2,i)*cos(phi) + b(1,i)*sin(phi))^2 - l(i)^2) ...
                * 2 * (r(2) - a(2,i) + b(2,i)*cos(phi) + b(1,i)*sin(phi));
    Gphi(i) = 2*(2*(b(2,i)*cos(phi) + b(1,i)*sin(phi))*(a(1,i) - r(1) - b(1,i)*cos(phi) + b(2,i)*sin(phi)) + 2*(b(1,i)*cos(phi) - b(2,i)*sin(phi))*(r(2) - a(2,i) + b(2,i)*cos(phi) + b(1,i)*sin(phi)))*((a(1,i) - r(1) - b(1,i)*cos(phi) + b(2,i)*sin(phi))^2 + (r(2) - a(2,i) + b(2,i)*cos(phi) + b(1,i)*sin(phi))^2 - l(i)^2);
end
G = [sum(Gx);sum(Gy);sum(Gphi)];
end