function [f, flag] = Optimal_ForceDistributions(A,w_c,m_p,f_min,f_max,f_ref, f_prev)

% Kernel Translation V2.
%
% A: Structure matrix
% w_c: Desired Wrench applied on the platform
% f_min: Minimum Cable Tension
% f_max: Maximum Cable Tension
% f_ref: Desired/reference Cable Tension
%
% Flag Value Representations:
% Flag = 0: Acceptable Solution
% Flag = 1: Kernel has different signs, uncontrollable 
% Flag = 2: No acceptable step length
%
% Elias Olsen Almenningen & Magnus Grøterud

% if isempty(f_prev)
%     f_prev = [0 0 0 0]';
% end
%% Kernel Translation V2: return of the dumbfuckery 

% Initialize Flag
flag = 0;

%% Calculate the kernel h of A^T
m = length(A);
A_t = A';
% [~, ~, V] = svd(A_t);
% h = V(:, end)
h = null(A_t);
A_pseudo = pinv(A_t);       % Pseudo-Inverse of structure matrix

%% Calculate wrench
wp = [0;-m_p*9.81;0];       % External wrench affecting the platform
w = -w_c + wp;                 % Necessary wrench to generate w_c, while counteracting gravity              

%% Calculate Feasible Solutions
% fm = 0.5*(f_min*ones(4,1) + f_max*ones(4,1));                   % Medium Feasible Cable Force
f0 = -(A_pseudo*w);                                             % Arbitrarily Solution on the line

if (min(f0) > 0) && (max(f0) <= f_max) && (min(f0) >= f_min)      % If arbitrarily solution is wrench-feasible
    f = f0;
else                                                            % Else, need to move it 
    % Check if signs are equal (If we can "move" the cabel forces along the nullspace)
    if ~all(sign(h) == sign(h(1)))
        flag = 1;
        f = f_prev;
        return;
    end
    
    % Calculate Step Sizes
    lambda_l = zeros(m,1);          % Lowest stepsize (Some Cables on f_min)
    lambda_h = zeros(m,1);          % Highest stepsize (Some cables on f_max)
    lambda_r = zeros(m,1);          % Desired stepsize (Preferred cable tension )
    
    for i = 1:m
        lambda_l(i) = (f_min - f0(i))/h(i);
        lambda_h(i) = (f_max - f0(i))/h(i);
        lambda_r(i) = (f_ref - f0(i))/h(i); % Test
    end
     
    lambda_min = max(lambda_l);
    lambda_max = min(lambda_h);
    lambda_ref = median(lambda_r);      
    lambda_mid = (lambda_min + lambda_max)/2;

    % If step sizes are not valid
    if lambda_min > lambda_max
        flag = 2;
        disp("Step sizes are inconsistent. Returning to home pos.")
        f = f_ref*ones(4,1);
    else
        % Calculate preferred cable force
        f = f0 + h*lambda_ref;
        % % Check if reference cable tension is feasible, else change it
        % if min(f) < f_min
        %     f = f0 + h*lambda_min;
        % elseif max(f) > f_max
        %     f = f0 + h*lambda_max;

        % If f is still not acceptable
        if (min(f) < f_min) || (max(f) > f_max)             % If the solution is not acceptable
            f = f_prev;                                     % No solution, return to center of frame
            flag = 3;                                       % Set flag = 2 to alert
            disp("min(f) < f_min) or (max(f) > f_max")
            h
            lambda_min
            lambda_mid
            lambda_max
            f0
        end

    end

end
    
    
                                               
    
    
end

