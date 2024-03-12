function writeParameter(device, parameter_string, value)

command = "w" + " " + parameter_string + " " + string(value);
writeline(device, command)

end