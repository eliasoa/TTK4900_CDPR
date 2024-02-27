function info = getNumberOfAxisAndCOMPortNumber()
% Returns a vector containg the ODrive number of motor controllers connected to
% the pc and the number of their respective COM port. 
% Assuming all USB Serial Devices are motor controllers, and that the
% lowest COM port corresponds to ODrive 0 and so on

devices = IDSerialComs();

% Initialize an array to hold the results
usbDevices = {};

% Count the number of USB Serial Devices
usbDeviceCount = 0;

for i = 1:size(devices, 1)
    if ischar(devices{i, 1}) && strcmp(devices{i, 1}, 'USB Serial Device')
        % Add the device and its number to the results array
        usbDevices(end+1, :) = {devices{i, 1}, devices{i, 2}};
        usbDeviceCount = usbDeviceCount + 1;
    end
end

% Generate a list from 0 to one less than the number of USB Serial Devices
axis = 0:(usbDeviceCount - 1);
% Extract COM port number from usbDevices
comPort = usbDevices(:,2);
% Generate the vector containg ODrive number id and its respective COM port
info = [axis', cell2mat(comPort)];




function devices = IDSerialComs()
% IDSerialComs identifies Serial COM devices on Windows systems by friendly name
% Searches the Windows registry for serial hardware info and returns devices,
% a cell array where the first column holds the name of the device and the
% second column holds the COM number. Devices returns empty if nothing is found.
%
% Source:
% Written by Benjamin Avants
% https://se.mathworks.com/matlabcentral/fileexchange/45675-identify-serial-com-devices-by-friendly-name-in-windows


devices = [];
Skey = 'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM';
[~, list] = dos(['REG QUERY ' Skey]);
if ischar(list) && strcmp('ERROR',list(1:5))
    disp('Error: IDSerialComs - No SERIALCOMM registry entry')
    return;
end
list = strread(list,'%s','delimiter',' '); %#ok<FPARK> requires strread()
coms = 0;
for i = 1:numel(list)
    if strcmp(list{i}(1:3),'COM')
        if ~iscell(coms)
            coms = list(i);
        else
            coms{end+1} = list{i}; %#ok<AGROW> Loop size is always small
        end
    end
end
key = 'HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\';
[~, vals] = dos(['REG QUERY ' key ' /s /f "FriendlyName" /t "REG_SZ"']);
if ischar(vals) && strcmp('ERROR',vals(1:5))
    disp('Error: IDSerialComs - No Enumerated USB registry entry')
    return;
end
vals = textscan(vals,'%s','delimiter','\t');
vals = cat(1,vals{:});
out = 0;
for i = 1:numel(vals)
    if strcmp(vals{i}(1:min(12,end)),'FriendlyName')
        if ~iscell(out)
            out = vals(i);
        else
            out{end+1} = vals{i}; %#ok<AGROW> Loop size is always small
        end
    end
end
for i = 1:numel(coms)
    match = strfind(out,[coms{i},')']);
    ind = 0;
    for j = 1:numel(match)
        if ~isempty(match{j})
            ind = j;
        end
    end
    if ind ~= 0
        com = str2double(coms{i}(4:end));
        if com > 9
            length = 8;
        else
            length = 7;
        end
        devices{i,1} = out{ind}(27:end-length); %#ok<AGROW>
        devices{i,2} = com; %#ok<AGROW> Loop size is always small
    end
end
end
end