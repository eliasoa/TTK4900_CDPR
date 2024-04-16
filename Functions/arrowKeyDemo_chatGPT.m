% FROM CHATGPT og mæggærn
function arrowKeyDemo_chatGPT(device)
    
    load("ODriveEnums.mat")
    % Initialize variables
    x   = 0;                % Desired x-position
    y   = 0;                % Desired y-position
    phi = 0;                % Desired phi-angle [radians]  
    escapePressed = false;  % Initialize termination button (Press Esc to )

    L               = 2.0;      % Length of platform
    posIncrement    = 0.5;      % Position Increment each arrow click
    angleIncrement  = 2;        % Angle increment each arrow click
    xRange          = 20;       % Width of frame
    yRange          = 20;       % Height of frame


    % Plot initial point
    figure;
    plot(x, y, 'o'); 
    hold on;
    plot([x x+0.5*L*cosd(phi)], [y y+0.5*L*sind(phi)])
    plot([x x-0.5*L*cosd(phi)], [y y-0.5*L*sind(phi)])
    hold off;

    title('Arrow Key Demo');
    xlim([-xRange/2, xRange/2])
    ylim([-yRange/2, yRange/2])
    grid on;
    
    % Set motor in drive state 
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_CLOSED_LOOP_CONTROL, device)

    % Wait for arrow key press
    while escapePressed == false
        key = waitforbuttonpress;
        
        % Check if the key press is valid
        if key == 1
            charPressed = get(gcf, 'CurrentCharacter');
            
            % Check which arrow key is pressed
            switch charPressed
                case 28 % Left arrow
                    x = x - posIncrement;
                case 29 % Right arrow
                    x = x + posIncrement;
                case 30 % Up arrow
                    y = y + posIncrement;
                case 31 % Down arrow
                    y = y - posIncrement;
                case 27 % Escape key
                    escapePressed = true;
                case 'a' % 'a' key
                    phi = phi + deg2rad(angleIncrement);
                case 'd' % 'd' key
                    phi = phi - deg2rad(angleIncrement);
                    
            end
            
            % Update the plot
            clf;
            plot(x, y, 'o'); 
            hold on;
            plot([x x+0.5*L*cos(phi)], [y y+0.5*L*sin(phi)])
            plot([x x-0.5*L*cos(phi)], [y y-0.5*L*sin(phi)])
            hold off;
            title('Arrow Key Demo');
            xlim([-xRange/2, xRange/2])
            ylim([-yRange/2, yRange/2])
            grid on;

            % Robot Control
            q_d = [x;y;phi];    % Desired state

            % % INSERT MOTOR CONTROLLER (CONTROLLER IS NOT COMPLETED)
            % [t1,t2,t3,t4] = CDPR_controller(q_d)

            % % INSERT WRITING TORQUE TO DRIVER (NOT DONE, NEED TO ADD DRIVER COMS TO INPUT)
            % setMotorTorque(t1, DEVICE1)
            % setMotorTorque(t2, DEVICE2)
            % setMotorTorque(t3, DEVICE3)
            % setMotorTorque(t4, DEVICE4)

            % % TEST
            % setMotorPosition(x, 2, 0.5, device)

        end
    end
    % Terminate arrowkeyDemo
    close all
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, device)
end