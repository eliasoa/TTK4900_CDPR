function setEncoderPositions(device)
command = "w axis0.pos_estimate 0";
writeline(device,command);
end