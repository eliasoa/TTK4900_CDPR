function A = WrenchMatrix_V2(a,b,q)
% A = WrenchMatrix(a,b,q)
% WrenchMatrix calculates the transpose of the structure matrix 
% Equation 3.5 in Cable-Driven Parallel Robots 
% by Andreas Pott
%
% Inputs:
% a: vector of proximal attachment points in {i} on the form
%    a   =   | a1_x ... am_x |
%            | a1_y ... am_y |
%
% b: vector of distal attachment points in {b} on the form 
%    b   =   | b1_x ... bm_x |
%            | b1_y ... bm_y |
%
% q: vector with pose of the platform in {i} containg the carthesian
%    position vector r and the angle phi denoting the roation about the 
%    z axis [rad]
%
%    q   =   |  r  | =  |  x  | 
%            | phi |    |  y  | 
%                       | phi | 
%
%
% Outputs:
% A: matrix containing unit vector and cross product (in 2D) between distal
% attachment point and unit vector
%
%    A  =   | u_1 ... b_1 X u_1 |
%           |  .   .      .     |
%           |  .   .      .     |
%           |  .   .      .     |
%           | u_m ... b_m X u_m |
%
%
% Author:    Elias Olsen Almenningen & Magnus Grøterud
% Date:      13 March 2024
% Revisions: 


% Parameters and variables
m           = length(a);                % number of cables 
r           = q(1:2);                   % carthesian position of MP
phi         = q(3);                     % orientation of the platform [rad]
R           = [ cos(phi) -sin(phi);     % rotation matrix from {i} to {b} [SO3]
                sin(phi)  cos(phi)];

% Memory allocation
L = zeros(2, m);
l = zeros(m, 1);
u = zeros(2, m);

% Rotated Platform Anchor Points
b_F = zeros(2,length(b));
for i = 1:m
   b_F(:,i) = R*b(:,i); 
end

% Cable Vectors
for i = 1:m
    L(:,i) = a(:,i) - r - b_F(:,i);
end
% Cable Lengths
for i = 1:m
    l(i) = norm(L(:,i));
end

% Unit Cable Vectors
for i = 1:m
    u(:,i) = L(:,i)/l(i);
end

% Cross-product in 2 dimensions
h           = zeros(1,m);
for i = 1:m
    h(i) = b_F(1,i)*u(2,i) - b_F(2,i)*u(1,i);
end


% Assemble A matrix
A           = [ u(:,1)' h(1);
                u(:,2)' h(2);
                u(:,3)' h(3);
                u(:,4)' h(4)];
end