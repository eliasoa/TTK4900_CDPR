function f = ForceAllocIterative(A,a,b,f_min, f_max, f_ref, )
% Function for calculating optimal force distributions of a Cable Robot.
% Inspired by Einar Ueland, Thomas Sauder and Roger Skjetne, Department of
% Marine Technology; Norwegian University of Science and Technology; 
% Centre for Autonomous Marine Operations and Systems (NTNU AMOS);
% NO-7491 Trondheim, Norway
% SINTEF Ocean; NO-7465 Trondheim, Norway  
%

m = 3;          % Number of Controllable DOFs
n = 4;          % Number of actuating cables

W = A'; 
H = pinv(W);
f0 = f_ref;

% Params
sigma = norm(H,1);
lambda = max(abs(f_max - f0), abs(f0 - f_min));
delta

% Defining the objective function

g_f = 





