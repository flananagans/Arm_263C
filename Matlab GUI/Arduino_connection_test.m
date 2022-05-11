
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   MAE C263C Project
%   Keybot - The 1 finger speed racer, that can’t press shift.
%
%   Authors:        Will, Gabriel, Jake and Juan
%   Date:           5/3/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all; clc

%% Connect to Teensy
connectedTeensy = serialport("COM6", 115200);

%% Startup and enable motors
while(connectedTeensy.NumBytesAvailable > 0) 
    connectedTeensy.readline()
end
connectedTeensy.writeline('EN');
%% Send a command
connectedTeensy.writeline('S_Q 0 0');
while(connectedTeensy.NumBytesAvailable > 0) 
    connectedTeensy.readline()
end

%% Disable and close files
connectedTeensy.writeline('DIS');
while(connectedTeensy.NumBytesAvailable > 0) 
    connectedTeensy.readline()
end
connectedTeensy.writeline('OFF');
while(connectedTeensy.NumBytesAvailable > 0) 
    connectedTeensy.readline()
end