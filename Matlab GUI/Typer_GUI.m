
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   MAE C263C Project
%   Keybot - The 1 finger speed racer, that can’t press shift.
%
%   Authors:        Will, Gabriel, Jake, and Juan
%   Date:           5/3/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear all; clc

%% Start up the GUI
keybot = KeybotInterface;
keybot.matlabFileTimer = 1; % Toggle to use file rather than app timer.
pause(2); %Setup pause.

%% Run loop to collect letters pressed
key_pressed = keybot.currLetter; % Get current pressed key
curr_key_count = keybot.keyCounter; % Get current keyboard click count
desired_string = keybot.TexttotypeTextArea.Value{1}; % Get string to be typed
desired_letter_pos = 1; %Start with the first letter

% Runs and listens to input until timer hits 0 or the "End" button is pressed.
while true 
    
    pause(1); % Helps the UI actually run. Lower means faster

    % Jake, call your function here, and I will wait for a key to be
    % pressed before I continue (That's what the next if statement does)

    % e.g. function_to_press_letter (desired_string(desired_letter))
    fprintf("Calling Jake's Function. Desired Letter: %s\n", desired_string(desired_letter_pos));
    
    if keybot.keyCounter == curr_key_count %&& keybot.timerActive == 1
        fprintf("Skipping Check, no key pressed. curr_key_count = %d\n", curr_key_count);
    elseif keybot.keyCounter > curr_key_count %&& keybot.timerActive == 1
        curr_key_count = keybot.keyCounter; % Increment key counter if key was pressed
        key_pressed = keybot.currLetter; % Get current pressed key
        if key_pressed == desired_string(desired_letter_pos)
            desired_letter_pos = desired_letter_pos + 1;
            fprintf("Success, moving to next letter. New desired Letter: %s\n", desired_string(desired_letter_pos));
        elseif key_pressed == "space" && desired_string(desired_letter_pos) == " "
            desired_letter_pos = desired_letter_pos + 1;
            fprintf("Successful 'space', moving to next letter. New desired Letter: %s\n", desired_string(desired_letter_pos));
           % Need to learn to deal with backspaces and odd keypresses like
           % "Enter". Will do this soon!
%         elseif key_pressed == "backspace" && backspace_needed > 0 " "
%             desired_letter_pos = desired_letter_pos - 1;
%             fprintf("Successful 'backspace', moving to previous letter. Old desired Letter: %s\n", desired_string(desired_letter_pos));
        end
        
    end

    % --------------------------------------------------------------------
    % ---------------- Below is simply for timer purposes ----------------
    % --------------------------------------------------------------------
    if keybot.timerActive == 1
        if keybot.timerStarted == 0
            tic; start_time = keybot.timeLeft; keybot.timerStarted = 1;
        end
        elapsed_time = toc;
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

fprintf("Game over!\n")
pause(5);
keybot.delete
