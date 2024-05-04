% 2DX3 - Deliverable (2) - Final Project
% Matthew Jarzynowski

clear;
clc;

global isRunning dataPoints serialObj;

comPort = 'COM3'; % % COM3 on laptop, COM7 on PC
baudRate = 115200; 

serialObj = serialport(comPort, baudRate);

% Flags to indicate running state and data buffer
isRunning = true;
dataPoints = [];

% Read settings
configureTerminator(serialObj, 'LF'); 
set(serialObj, "Timeout", 40); % Timeout delay, 4 mins, but not really

% Timer to manage data transfer, synced
timerObj = timer('TimerFcn', @timeoutCallback, 'StartDelay', 40, 'ExecutionMode', 'singleShot');
start(timerObj);

disp('Reading data, press (Ctrl + C) to stop transfer.'); % Information

% Loop to read current data from serial, write to "data.txt", and display
% current readings
while isRunning
    try
        
        % Read a line of data from the serial port
        data = readline(serialObj);
        disp(data); % Display the data in the terminal

        % Skip iteration if line is empty
        if data == ""
            continue;
        end

        % Convert the string data to numbers and store in the array
        newPoint = str2num(data); % Convert the string line to a numeric array
        
        % If the incoming data has a length 
        if length(newPoint) == 3
            dataPoints(end+1, :) = newPoint; % Add new data point to array
        end
        
        % Reset the timer if data is received
        stop(timerObj);
        start(timerObj);
    end
end

% Cleanup
delete(timerObj); % Delete timer object
delete(serialObj); % Delete serial object

clear global isRunning dataPoints serialObj;
disp('Program ended.'); % Information

% Define the timeout callback function
function timeoutCallback(~, ~)
    global isRunning dataPoints serialObj;
    
    % This is a warning for the user based on the serial timeout
    disp('No data received within 40 seconds. Writing collected data to data.txt and stopping.');

    % Open 'data.txt' for writing
    fileID = fopen('data.txt', 'w');
    if fileID ~= -1
        % Write each data point to 'data.txt' in X,Y,Z format
        for idx = 1:size(dataPoints, 1)
            fprintf(fileID, '%f,%f,%f\n', dataPoints(idx, :));
        end
        fclose(fileID);  % Close the file
        disp('Data successfully written to data.txt.');
        run('plot.m');
    else
        disp('Failed to open data.txt for writing.');
    end

    isRunning = false;  % Set the flag to false to stop the loop
    delete(serialObj);  % Cleanup serial object
end