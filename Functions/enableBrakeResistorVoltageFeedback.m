function enableBrakeResistorVoltageFeedback(device)
nominal_voltage = 24;
ramp_start = nominal_voltage + 2;
ramp_stop = nominal_voltage + 6;

% Enable dc_bus_voltage_feedback
command = 'w config.brake_resistor0.enable_dc_bus_voltage_feedback 1';
writeline(device,command);

% Apply suggested settings to dc_bus_voltage_feedback_ramp_end
command = 'w config.brake_resistor0.dc_bus_voltage_feedback_ramp_start ' + string(ramp_start);
writeline(device,command);

command = 'w config.brake_resistor0.dc_bus_voltage_feedback_ramp_end ' + string(ramp_stop);
writeline(device,command);
end