function [f_positive] = TranslationWorkspace_V2(phi_0,a,b, f_min,f_max, f_ref, w, r_p, resolution, color)
% Function for calculating the Translation Workspace of a Cable-Driven
% Parallel Robot. Inspired by the work presented by Pott in "Cable-Driven
% Parallel Robots: Theory and Application", p. 161.
%
% Inputs:
%   phi_0       : Scalar, Orientation of the mobile platform [rad]
%   a           : 2x4 Matrix, ... Anchor Points for cable attachment 
%   b           : 2x4 Matrix, Body Anchor Points for cable attachment 
%   f_min       : Scalar, Minimum Limit on cable force
%   f_max       : Scalar, Maximum Limit on cable force
%   f_ref       : Scalar, Desired cable force 
%   w           : 3x1 Vector, Desired Wrench
%   r_p         : Scalar, Radius of pulleys
%   resolution  : Scalar, numb
%   color       : Color of the visualization plot (ex: "red")
%
% Outputs:
%
%  
%   
%
% Created by Magnus Grøterud

% Extract Lengths of the Base
x_dim = norm(a(:,3) - a(:,2));
y_dim = norm(a(:,2) - a(:,1));


% Grid of the "Base Workspace"
x_grid = linspace(-x_dim/2, x_dim/2, resolution);
y_grid = linspace(-y_dim/2, y_dim/2, resolution);

X = length(x_grid);
Y = length(y_grid);

% Allocations for solutions
f_positive = zeros(X,Y);


f_prev = f_ref*ones(4,1);

for i=1:X
    for j=1:Y
        % Define current pose to analyze
        q = [x_grid(i); y_grid(j);phi_0];

        [~,betar] = p_inverse_kinematics(a,b,q, r_p);
        A_t = structure_matrix(a,b,q,r_p, betar);
        A = A_t';

        % Calculate Force Allocation 
        [~,~, flag]  = ForceAllocIterativeSlack(A,f_min, f_max, f_ref, f_prev, w);
        if flag == 0
            f_positive(i,j) = 1;
        end
    end
end
% Plotting

% Create a grid of coordinates from (0,0) to (1,1)
% [x, y] = meshgrid(linspace(0, 1, size(f_positive, 2)), linspace(0, 1, size(f_positive, 1)));

% [x,y] = meshgrid(x_grid, y_grid);
% Find the (x, y) coordinates where the matrix has a value of 1
[row, col] = find(f_positive == 1);

% Plot the points
scatter(x_grid(row), y_grid(col), 'filled', 'MarkerFaceColor', color); % 'filled' option fills the markers
grid on;
% Set axis labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('Translation Workspace, with $$\phi _0 = $$ ' + string(rad2deg(phi_0)) + ' degrees, $$w_{ref} = [$$'+ string(w(1)) + ',' + string(w(2)) + ',' + string(w(3)) + ']', 'Interpreter','latex');

% Plotting the shape created by points in matrix 'b'
hold on; % Keep the current plot
plot([b(1,:), b(1,1)], [b(2,:), b(2,1)], 'k-o', 'LineWidth', 2); % Plot shape as a closed loop
hold off;



% Optionally, set axis limits based on the matrix size
axis([-0.5, 0.5, -0.5, 0.5]);

% % Displaying the matrix where 1s are plotted and adjusting the axes
% imagesc(y_grid,x_grid, f_positive);
% colormap('gray'); % Setting colormap to grayscale for better visibility
% colorbar; % Displaying colorbar to indicate values
