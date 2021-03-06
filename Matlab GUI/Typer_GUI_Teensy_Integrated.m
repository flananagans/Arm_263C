
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   MAE C263C Project
%   Keybot - The 1 finger speed racer, that can’t press shift.
%
%   Authors:        Will, Gabriel, Jake, and Juan
%   Date:           5/3/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;

%% Connect to Teensy %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variables you probably want to change
port = "COM5"
desired_string = 'avalon avalon avalon'; % Chose what you want to be typed
% desired_string = 'avalon'; % Chose what you want to be typed
% desired_string = 'the quick brown fox jumps over the lazy dog'; %Full strings

connectedTeensy = serialport(port, 115200); %Need to select the right port

% Startup and enable motors
readAllData(connectedTeensy);
connectedTeensy.writeline('EN');

%% Start up the GUI
keybot = KeybotInterface;
keybot.matlabFileTimer = 1; % Toggle qto use file rather than app timer.
keybot.TexttotypeTextArea.Value = desired_string; % Get string to be typed

% Set initial pose of arm
connectedTeensy.writeline('S_K Q');
readAllData(connectedTeensy);


while(~keybot.timerActive)
    pause(0.5)
end

%% Run loop to collect letters pressed
key_pressed = keybot.currLetter; % Get current pressed key
curr_key_count = keybot.keyCounter; % Get current keyboard click count
desired_letter_pos = 1; %Start with the first letter
backspaces_needed = 0; %Initialize
focus(keybot.TypehereTextArea);
stopwatch_tic = tic; fprintf("Starting timer\n"); % Just in case

% Send the first letter. Robot starts moving, and we enter the loop
tempcmd = sprintf("connectedTeensy.writeline('S_K %s');", desired_string(desired_letter_pos));
eval(tempcmd);
fprintf("Initial desired Letter: '%s'.\n", desired_string(desired_letter_pos));

% Runs and listens to input until timer hits 0 or the "End" button is pressed.
while true 
    
    pause(0.001); % Helps the UI actually run. Lower means faster
    
    % We only go into this loop whenever a key is pressed.
    if keybot.keyCounter == curr_key_count %&& keybot.timerActive == 1
%         fprintf("Skipping Check, no key pressed. curr_key_count = %d\n", curr_key_count);
    elseif keybot.keyCounter > curr_key_count %&& keybot.timerActive == 1
        curr_key_count = keybot.keyCounter; % Increment key counter if key was pressed
        key_pressed = keybot.currLetter; % Get current pressed key
        if backspaces_needed == 0
            if key_pressed == desired_string(desired_letter_pos)
                desired_letter_pos = desired_letter_pos + 1;                
                if desired_letter_pos > length(desired_string); fprintf("Good work. You're done!\n"); keybot.timerActive = 0; break; end % Check if you won!
                fprintf("Success, moving to next letter. New desired Letter: '%s'.\n", desired_string(desired_letter_pos));
            elseif key_pressed == "space" && desired_string(desired_letter_pos) == " "
                desired_letter_pos = desired_letter_pos + 1;
                if desired_letter_pos > length(desired_string); fprintf("Good work. You're done!\n"); keybot.timerActive = 0; break; end % Check if you won!
                fprintf("Successful 'space', moving to next letter. New desired Letter: '%s'.\n", desired_string(desired_letter_pos));
            elseif key_pressed == "backspace" && backspaces_needed == 0
                desired_letter_pos = desired_letter_pos - 1;
                fprintf("Unsuccessful 'backspace', moving to previous letter. New desired Letter: '%s'.\n", desired_string(desired_letter_pos));
            else
                backspaces_needed = backspaces_needed + 1;
                fprintf("Wrong letter, backspaces_needed = %d\n", backspaces_needed);
            end
        elseif key_pressed == "backspace" && backspaces_needed > 0
                backspaces_needed = backspaces_needed -1;
                if backspaces_needed > 0
                    fprintf("Additional backspaces_needed = %d\n", backspaces_needed);
                elseif backspaces_needed == 0
                    fprintf("Successful 'backspaces', moving to previous letter. New desired Letter: '%s'.\n", desired_string(desired_letter_pos));
                end
        else 
            backspaces_needed = backspaces_needed + 1;
            fprintf("Wrong letter, backspaces_needed = %d\n", backspaces_needed);
        end
    
        % Here we tell the robot to press the desired_letter
        if backspaces_needed > 0
            tempcmd = sprintf("connectedTeensy.writeline('S_K 2');");
            fprintf("Told robot to backspace\n");
            eval(tempcmd);
        elseif backspaces_needed == 0
            if desired_string(desired_letter_pos) == " "
                tempcmd = sprintf("connectedTeensy.writeline('S_K 1');");
                eval(tempcmd);
                fprintf("Told robot to hit: 'space'.\n");
            else
                tempcmd = sprintf("connectedTeensy.writeline('S_K %s');", desired_string(desired_letter_pos));
                eval(tempcmd);
                fprintf("Told robot to hit: '%s'.\n", desired_string(desired_letter_pos));
            end
        else
            error("backspaces_needed is negative")
        end
%             readAllData(connectedTeensy);
    end

    % --------------------------------------------------------------------
    % ---------------- Below is simply for timer purposes ----------------
    % --------------------------------------------------------------------
    if keybot.timerActive == 1
        if keybot.timerStarted == 0
            timer_tic = tic; start_time = keybot.timeLeft; keybot.timerStarted = 1;
        end
        elapsed_time = toc(timer_tic);
        if keybot.timeLeft > 0
            keybot.timeLeft = start_time - elapsed_time;
        else
            keybot.timerActive = 0;
            keybot.timerStarted = 0;
            keybot.TimeLeftsecEditField.Value = "0";
        end
        keybot.TimeLeftsecEditField.Value = num2str(floor(keybot.timeLeft));
    end

    if keybot.timeLeft == 0; break; end % End the loop once time reaches 0.
end
endtime = toc(stopwatch_tic);
fprintf("Game over! Time_elapsed = %.2f.\n", endtime);
pause(2); % Pause to read time.

%% End connection with Teensy: Disable and close files %%%%%%%%%%%%%%%%%%%
connectedTeensy.writeline('OFF');
readAllData(connectedTeensy);

% Close everything
keybot.delete;
connectedTeensy = [];

fprintf("Game over! Time_elapsed = %.2f.\n", endtime);

%% Functions
function readAllData(connectedTeensy) 
%Read All Data. Reads incoming data from Teensy until no bytes left.
    while(connectedTeensy.NumBytesAvailable > 0) 
        fprintf(connectedTeensy.readline());
    end
end
