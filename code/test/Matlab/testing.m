% Clear all variables
clear;

% Create serial object
ser = serial('COM4', 'BaudRate', 115200);
set(ser, 'Parity', 'none');

% Open connection
fopen(ser);

% Wait for startup
pause(5);

% Send initialization data
fprintf('%d %d %f %f %f', 900, 0, 0.52, 0.01, 0);

% Wait for a moment to ensure command received
pause(1);

% Setup timing
polling_freq_hz = 66.67;
polling_time_s = 30;

t = 0;
t_max = round(polling_time_s / (1 / polling_freq_hz));

% Initialize variables
pitch = zeros(0, t_max);

figure(1)
hold on;
title('Multicopter Pitch');
xlabel('Time');
ylabel('Angle (deg)');
ylim([-50 50]);
xlim([0 t_max]);
lHandle = line(NaN, NaN);

% Loop and receive data until time has been met
while t < t_max
    % Increment time
    t = t + 1; 
    
    % Grab received data and parse    
    pitch(t) = fscanf(ser, '%f')
        
    X = get(lHandle, 'XData');
    Y = get(lHandle, 'YData');
    
    X = [X t];
    Y = [Y pitch(t)];
    
    set(lHandle, 'XData', X, 'YData', Y);
    
    pause(1 / polling_freq_hz);
end

% Close and delete serial connection
fclose(ser);
delete(ser);