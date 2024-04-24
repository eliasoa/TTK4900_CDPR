% Test area
clear;
%% Set up ASCII communication
baudrate = 115200;
timeout = 1;
ODriveStruct = initSerialPorts(baudrate, timeout);
%%

for i = 1:100
    t_funcStart = tic;
    % vel = TEST_getEncoderVelocity(ODriveStruct);
    % pos = TEST_getEncoderPosition(ODriveStruct);
    % [pos, vel] = TEST_getEncoderReading(ODriveStruct);
    getMotorPosition(ODriveStruct.ODrive0);
    getMotorPosition(ODriveStruct.ODrive1);
    getMotorPosition(ODriveStruct.ODrive2);
    getMotorPosition(ODriveStruct.ODrive3);
    t_tot(i) = toc(t_funcStart);
end
%%
plot(t_tot)