%% Lab2 - Transfer Function Simulation Version
%% Starting Code 
clear all; clc; close all

% Input the speed of the rover
prompt_box = {'Enter Testing Mode: (heading, speed, waypoint)', 'Enter the Speed of the Rover (mm/s)   min: 70, max: 200'};
dlgtitle = 'Rover Commands';
dims = [1 70];
definput = {'speed', '100'};
answer = inputdlg(prompt_box,dlgtitle,dims,definput);
check = 0;

% mode_1 = heading control; mode_2 = speed control; mode_3 = waypoint
mode_1 = 0; mode_2 = 0; mode_3 = 0;
if str2double(answer(2)) >= 70 && str2double(answer(2)) <= 200
    check = check + 1;
end
if string(answer(1)) == "heading"
    check = check + 1;
    mode_1 = 1;
elseif string(answer(1)) == "speed"
    check = check + 1; 
    mode_2 = 1;
elseif string(answer(1)) == "waypoint"
    check = check + 1; 
    mode_3 = 1;
else
    error('Enter appropriate testing mode!')
end
if check < 2
    error('Check inputs! Enter inputs with appropriate ranges')
end
wr.forward_spd = str2double(answer(2));

% loading vehicle parameters
vhc_param;
wr.heading_vec = [1,1];

%% Transfer Function Setup
% Speed transfer function: output is actual speed (mm/s)
num_spd = [-11210 43120 60330];
den_spd = [1 961.2 17530 17990];
TF_speed = tf(num_spd, den_spd);

% Heading transfer function: output is heading angle (rad)
num_head = [0.4972 0.1716 0.3878 0.1714];
den_head = [1 0.7091 1.551 0.4573 0.5846 0];
TF_heading = tf(num_head, den_head);

% Simulation parameters
dt = 0.01; % 30ms sampling time (same as original)
simulation_time = 30; % 30 seconds simulation
time_steps = round(simulation_time / dt);

% Convert to discrete time for simulation
TF_speed_d = c2d(TF_speed, dt, 'zoh');
TF_heading_d = c2d(TF_heading, dt, 'zoh');

% Initialize simulation variables
wr.actual_speed = 0;        % Current actual speed (mm/s)
wr.actual_heading = 0;      % Current actual heading (rad)
wr.pos = [0, 0];           % Current position [x, y] (mm)
wr.pos_old = [0, 0];       % Previous position [x, y] (mm)

% Control input history for transfer functions (initialize with zeros)
u_speed_history = [0; 0];      % Speed control input history (start with 2 zeros)
u_heading_history = [0; 0];    % Heading control input history (start with 2 zeros)

% Output history for transfer functions  
y_speed_history = [];      % Speed output history
y_heading_history = [];    % Heading output history

% Logfile preparation
thismoment = clock;
date_time = '';
for i = 1:5
    date_time = [date_time, num2str(thismoment(i), '%02d')];
    if i == 3
        date_time = [date_time, '_'];
    end
end

if exist('lab_data', 'dir') ~= 7
    mkdir('lab_data');
end
file_name = ['lab_data/data_sim_',date_time,'.csv'];
fid = fopen(file_name,'w');

%% Read waypoints for waypoint control mode
if mode_3
    wr.WP = readmatrix("Rover_wp_lab2.xlsx", 'sheet','white_rover');
    wr.numWP = size(wr.WP,1);
end

%% Simulation loop
wr.disable = 0;
time.old = 0;
loop = 1;

% Initialize target for kill switch simulation (optional)
target_height = 600; % mm, simulated target height

fprintf('Starting simulation...\n');
tic
for step = 1:time_steps
    time.curr = toc;
    time.dt = time.curr - time.old;
    
    % Ensure minimum time step
    if time.dt < dt
        pause(dt - time.dt);
        time.curr = toc;
        time.dt = time.curr - time.old;
    end
    
    % Store old position for speed calculation
    wr.pos_old = wr.pos;
    
    %% Call the original controller functions
    if mode_1
        wr.heading_dir = [1,1];  % Reference direction (x-axis)
        % Create heading vector from current heading for compatibility with original function
        wr.heading_vec = [cos(wr.actual_heading), sin(wr.actual_heading)];
        wr = wr_control_heading(wr, time);
        fprintf('Step %d: Running heading test, heading = %.3f deg\n', step, (rad2deg(wr.actual_heading))); 
    elseif mode_2
        wr = wr_control_spd(wr, time);
        fprintf('Step %d: Running speed test, speed = %.1f mm/s\n', step, wr.actual_speed);
    else
        % Create heading vector from current heading for compatibility with original function
        wr.heading_vec = [cos(wr.actual_heading), sin(wr.actual_heading)];
        wr.heading_dir = [0,0];
        wr = wr_control_wp(wr, time);
        fprintf('Step %d: Running waypoint navigation, pos = [%.1f, %.1f]\n', step, wr.pos(1), wr.pos(2));
        
    end

    %% Update transfer function simulation based on original controller outputs
    % Extract control signals from the original controller outputs
    PWML = double(wr.PWML);
    PWMR = double(wr.PWMR);
    DIRL = wr.DIRL;
    DIRR = wr.DIRR;
    
    % Convert PWM and direction to signed control inputs
    left_control = PWML * (2*DIRL - 1);   % Positive if forward, negative if backward
    right_control = PWMR * (2*DIRR - 1);  % Positive if forward, negative if backward
    
    % Speed control input (average of both wheels for forward motion)
    u_speed = (left_control + right_control) / 2;
    u_speed_history = [u_speed_history; u_speed];
    
    % Heading control input (difference between wheels for turning)
    u_heading = (left_control - right_control) / 2;
    u_heading_history = [u_heading_history; u_heading];
    
    % Simulate speed response
    y_speed = lsim(TF_speed_d, u_speed_history, (0:length(u_speed_history)-1)*dt);
    wr.actual_speed = max(0, y_speed(end)); % Speed cannot be negative
    
    % Simulate heading response
    y_heading = lsim(TF_heading_d, u_heading_history, (0:length(u_heading_history)-1)*dt);
    wr.actual_heading = wr.actual_heading + y_heading(end) * dt;  % Integrate heading rate to get heading
    
    %% Update position based on speed and heading
    % Simple kinematic model: position update based on speed and heading
    if time.dt > 0
        dx = wr.actual_speed * cos(wr.actual_heading) * time.dt / 1000; % Convert mm to m for calculation, then back
        dy = wr.actual_speed * sin(wr.actual_heading) * time.dt / 1000;
        wr.pos = wr.pos + [dx*1000, dy*1000]; % Convert back to mm
    end
    
    %% Check termination conditions
    % For waypoint mode, check if all waypoints are reached
    if mode_3 && wr.curWP > wr.numWP
        fprintf('All waypoints reached! Stopping simulation.\n');
        break;
    end
    
    % Simulated kill switch (target height condition)
    if step > time_steps * 0.8 % After 80% of simulation time, allow termination
        target_height = target_height - 10; % Simulate target moving down
    end
    
    if target_height < 500
        wr.disable = wr.disable + 1;
    end
    
    if wr.disable > 50 % Equivalent to 1.5 seconds
        fprintf('Kill switch activated! Stopping simulation.\n');
        break;
    end
    
    %% Logging
    data_to_log = [time.curr, wr.pos(1), wr.pos(2), wr.actual_speed, wr.actual_heading];
    if loop == 1
        fprintf(fid, 'time[s], wr_x[mm], wr_y[mm], speed[mm/s], heading[rad]\n');
        format_string = '';
        for i=1:1:length(data_to_log)
            format_string = strcat(format_string, '%3.4f');
            if i < length(data_to_log)
                format_string = strcat(format_string, ',');
            else
                format_string = strcat(format_string, '\n');
            end
        end
    end
    fprintf(fid, format_string, data_to_log);
    loop = loop + 1;
    time.old = time.curr;
end

%% End simulation
fprintf('Simulation completed!\n');
fclose(fid);

% Plot results
figure;
subplot(2,2,1);
plot_data = readmatrix(file_name);
plot(plot_data(:,1), plot_data(:,4));
xlabel('Time (s)');
ylabel('Speed (mm/s)');
title('Speed vs Time');
grid on;

subplot(2,2,2);
plot(plot_data(:,1), plot_data(:,5));
xlabel('Time (s)');
ylabel('Heading (rad)');
title('Heading vs Time');
grid on;

subplot(2,2,3);
plot(plot_data(:,2), plot_data(:,3));
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
title('Robot Trajectory');
grid on;
axis equal;

subplot(2,2,4);
plot(plot_data(:,1), plot_data(:,2), plot_data(:,1), plot_data(:,3));
xlabel('Time (s)');
ylabel('Position (mm)');
title('Position vs Time');
legend('X', 'Y');
grid on;