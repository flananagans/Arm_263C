
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   MAE C263C Project
%   Keybot - The 1 finger speed racer, that can’t press shift.
%
%   Authors:        Will, Gabriel, Jake and Juan
%   Date:           5/3/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all; clc

%% Connect to Teensy
port = "COM5";
connectedTeensy = serialport(port, 115200); %Need to select the right port

%% Startup and enable motors
readAllData(connectedTeensy);
connectedTeensy.writeline('EN');

%% Send a command
connectedTeensy.writeline('S_Q 0 0');
readAllData(connectedTeensy);

%% Disable and close files
connectedTeensy.writeline('DIS');
readAllData(connectedTeensy);
connectedTeensy.writeline('OFF');
readAllData(connectedTeensy);
fprintf("done\n")

%% Functions
function readAllData(connectedTeensy) 
%Read All Data. Reads incoming data from Teensy until no bytes left.
    while(connectedTeensy.NumBytesAvailable > 0) 
        connectedTeensy.readline()
    end
end