function T = getThermistorTemp(device)

command = "r axis0.motor.motor_thermistor.temperature";

writeline(device, command)

T = readline(device);
T = str2double(T);

end