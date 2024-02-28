% FROM CHATGPT og mæggærn
function arrowKeyDemo_chatGPT(device)

    load("ODriveEnums.mat")
    % Initialize variables
    x = 0;
    y = 0;
    escapePressed = false;

    % Plot initial point
    figure;
    plot(x, y, 'o');
    title('Arrow Key Demo');
    xlim([-20, 20])
    ylim([-20, 20])
    grid on;
    
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
                    x = x - 1;
                case 29 % Right arrow
                    x = x + 1;
                case 30 % Up arrow
                    y = y + 1;
                case 31 % Down arrow
                    y = y - 1;
                case 27 % Escape key
                    escapePressed = true;
            end
            
            % Update the plot
            clf;
            plot(x, y, 'o');
            title('Arrow Key Demo');
            xlim([-20, 20])
            ylim([-20, 20])
            grid on;

            setMotorPosition(x, 2, 0.5, device)

        end
    end
    close all
    setAxisState(ODriveEnums.AxisState.AXIS_STATE_IDLE, device)
end