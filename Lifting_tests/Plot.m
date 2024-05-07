% Define the filename of the CSV file
filename = "1kg/ODrive2/Down05.csv";

% Read the CSV file into a table
data = readtable(filename);

% Extract each column into a separate vector
time = data.time;
ODrive0Position = data.ODrive0Position;
ODrive1Position = data.ODrive1Position;
ODrive2Position = data.ODrive2Position;
ODrive3Position = data.ODrive3Position;
ODrive0Velocity = data.ODrive0Velocity;
ODrive1Velocity = data.ODrive1Velocity;
ODrive2Velocity = data.ODrive2Velocity;
ODrive3Velocity = data.ODrive3Velocity;
ODrive0Current = data.ODrive0Current;
ODrive1Current = data.ODrive1Current;
ODrive2Current = data.ODrive2Current;
ODrive3Current = data.ODrive3Current;

Tq = 8.27/150;
x0 = 0
x1 = inf

figure(1)
subplot(3,1,1)
plot(time,ODrive2Position)
xlim([x0 x1])
subplot(3,1,2)
plot(time,ODrive2Current*Tq);
xlim([x0 x1])
title("torqs")
subplot(3,1,3)
plot(time,ODrive2Velocity);
title("vel")
% ylim([-3 0.6])
xlim([x0 x1])

t = 17.4;
% 722
% 4158
%%
averageValue = mean(ODrive2Current(319:881)*Tq)