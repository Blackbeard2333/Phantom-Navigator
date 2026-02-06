% Number of Simulations
% N = 10;
n_initializations = 50;
N = n_initializations;
set(groot, 'DefaultAxesFontName', 'Times New Roman');
% set(groot, 'DefaultAxesFontSize', 14); % Set font size for axes labels
% set(groot, 'DefaultTextFontSize', 14); % Set font size for text annotations
% set(groot, 'DefaultLegendFontSize', 12); % Set font size for legend

% MATLAB Time-related Parameters
Time = 400;
sampleTime = 0.01;
T = Time * (1 / sampleTime) + 1;
numSteps = Time * (1 / sampleTime) + 1;
time = sampleTime * (0:numSteps - 1);
time = time';

% Attacker identification and tracking model
attack_pos_noise_level = 0.5;
attack_vel_noise_level = 0.01;


% System profiles
% Physical parameters
quad_model;
Quad_Model = struct(...
    'Ixx', 0.016, ...
    'Iyy', 0.016, ...
    'Izz', 0.0274, ...
    'Im', 3.789e-6, ...
    'mass', 1, ...
    'g', 9.8);

% Sensor profiles
system_noise = 0.1;
gps_pos_noise = 0.5;
gps_vel_noise = 0.01;

imu_acc_noise = sqrt(0.0012356);
imu_gyro_noise = sqrt(deg2rad(0.025));
imu_mag_noise = sqrt(0.3 / sqrt(50));

% Anomaly detector statistics

% % % Parameters for gps noise 0.5
Gamma = timeseries(2.04, time);
Delta_1 = timeseries(3.4, time);
Delta_2 = timeseries(0.75, time);

% % Parameters for gps noise 2.5
% Gamma = timeseries(10.2, time);
% Delta_1 = timeseries(100.4, time);
% Delta_2 = timeseries(5.1, time);

% Initialize variables to store results
s_norm_value = 10;
s_norm_limit = s_norm_value * ones(length(time), 1);
S_Norm = timeseries(s_norm_limit, time);

delta_s_value = 2.5;
delta_s_limit = delta_s_value * ones(length(time), 1);
Delta_S_Lim = timeseries(delta_s_limit, time);

% Define spoofing target
spoofing_target_x = 1000;
spoofing_target_y = 800;

% Create a matching-size array
spoofing_target = [spoofing_target_x * ones(length(time), 1), ...
                   spoofing_target_y * ones(length(time), 1)];

% Convert to timeseries
Spoofing_Target = timeseries(spoofing_target, time);

% Define the time points
% T_waypoints = [0, 50, 100, 150, 200];
T_waypoints = [0, 400];

% % Define the waypoints corresponding to the time points
% Waypoints = [0, 0, 10;
%              100, 0, 10;
%              100, 100, 20;
%              0, 100, 20;
%              0, 0, 10]; % Repeat the last waypoint to match the time vector size

% Define the waypoints corresponding to the time points
Waypoints = [0, 0, 10; 2000, 0, 10]; % Repeat the last waypoint to match the time vector size

% % Define the waypoints corresponding to the time points
% Waypoints = [0, 0, 10;
%              0, 20, 10;
%              0, 40, 10;
%              0, 60, 10;
%              0, 80, 10]; % Repeat the last waypoint to match the time vector size

% Create the timeseries object
WaypointSeries = timeseries(Waypoints, T_waypoints);


% Noise passed to Simulink
attack_pos_noise = attack_pos_noise_level * randn(numSteps, 3);
attack_vel_noise = attack_vel_noise_level * randn(numSteps, 3); 
attacker_combined_noise = [attack_pos_noise, attack_vel_noise];
Attacker_Noise = timeseries(attacker_combined_noise, time);

GPS_Position_Noise = timeseries(gps_pos_noise * randn(numSteps, 3), time);
GPS_Velocity_Noise = timeseries(gps_vel_noise * randn(numSteps, 3), time);

IMU_Acc_Noise = timeseries(imu_acc_noise * randn(numSteps, 3), time);
IMU_Gyro_Noise = timeseries(imu_gyro_noise * randn(numSteps, 3), time);
IMU_Mag_Noise = timeseries(imu_mag_noise * randn(numSteps, 3), time);

System_Noise = timeseries(system_noise * randn(numSteps, 12), time);


% Run Simulink
sim('Quad_PID_FUSION_Close_Loop.slx');

% Extract data from simulink
time_data = squeeze(logsout.get('pos_x').Values.Time); % Extract timestamps

pos_x_visualize = squeeze(logsout.get('pos_x').Values.Data);
pos_y_visualize = squeeze(logsout.get('pos_y').Values.Data);

pos_x_fusion = squeeze(logsout.get('pos_x_fused').Values.Data);
pos_y_fusion = squeeze(logsout.get('pos_y_fused').Values.Data);

% Attacker predicted drone pose
attacker_pos_estimate = logsout.get('attacker_pos_obs').Values.Data;
attacker_pos_estimate_flattened = reshape(attacker_pos_estimate, 1, []);
attacker_vel_estimate = logsout.get('attacker_vel_obs').Values.Data;
attacker_vel_estimate_flattened = reshape(attacker_vel_estimate, 1, []);

% Find the first index where the position is within 5m of the target
distances = sqrt((pos_x_visualize - spoofing_target_x).^2 + (pos_y_visualize - spoofing_target_y).^2);
idx_attack_success = find(distances < 5, 1);  % Find first occurrence
[~, idx_attack_closest] = min(distances); % Find the closest point

% Anomaly statistics 
lambda_c = logsout.get('S_out').Values.Data;
lambda_r = logsout.get('r').Values.Data;

c_alarm = logsout.get('c_alarm').Values.Data;
r_alarm = logsout.get('r_alarm').Values.Data;

% Calculate statistics
% c_rate = sum(c_alarm) / T * 100
% r_rate = sum(r_alarm) / T * 100


if ~isempty(idx_attack_success)
    c_rate = sum(c_alarm(1:idx_attack_success)) / idx_attack_success * 100
    r_rate = sum(r_alarm(1:idx_attack_success)) / idx_attack_success * 100
else
    % If idx_attack_success is empty (no attack success), compute over all data
    c_rate = sum(c_alarm) / T * 100
    r_rate = sum(r_alarm) / T * 100
end


%% Drone Position Plot
figure(1);
clf;
hold on; % Allow multiple plots on the same figure



% Plot the raw and fused positions in 2D
plot(pos_x_visualize, pos_y_visualize, 'r', 'LineWidth', 1.5); % Raw position (Red)
plot(pos_x_fusion, pos_y_fusion, 'b', 'LineWidth', 1.5); % Fused position (Blue)
plot(spoofing_target_x, spoofing_target_y, 'mx', 'MarkerSize', 20, 'LineWidth', 2); % Spoofing Target (Pink "X")
plot([0, 2000], [0, 0], 'k--', 'LineWidth', 2); % Dashed black line

% Initialize legend labels as a cell array
legend_labels = {'Actual Position (Attacked)', 'Fused Position (Deceived)', 'Spoofing Target', 'Mission Trajectory'};

% Mark the first point within 5m of the target
if ~isempty(idx_attack_success)
    attack_success_x = pos_x_visualize(idx_attack_success);
    attack_success_y = pos_y_visualize(idx_attack_success);
    attack_success_time = time_data(idx_attack_success);  % Get the corresponding time

    attack_success_x_deceived = pos_x_fusion(idx_attack_success);
    attack_success_y_deceived = pos_y_fusion(idx_attack_success);

    plot(attack_success_x, attack_success_y, 'mo', 'MarkerSize', 10, 'LineWidth', 2); % Mark first close point (Pink "O")
    plot(attack_success_x_deceived, attack_success_y_deceived, 'bo', 'MarkerSize', 10, 'LineWidth', 2); % Mark first close point (Blue "O")

    % Append the first close point information to the legend
    legend_labels = [legend_labels, {sprintf('Attack Success Point (Attacked) (Time: T_{AS}=%.2f s)', attack_success_time)}];
    legend_labels = [legend_labels, {sprintf('Attack Success Point (Deceived) (Time: T_{AS}=%.2f s)', attack_success_time)}];
end

attack_closest_time = time_data(idx_attack_closest)
attack_closest_distance = min(distances)

% Labels and title
xlabel('Position X');
ylabel('Position Y');
xlim([0, 2000]);
ylim([-1000, 1000]);
title('Drone 2D Position');
grid on; % Turn on the grid for better visualization

% Add legend for clarity
legend(legend_labels, 'Location', 'best');

hold off; % Release hold after plotting


%% Anomaly Detector Statistics Plot (Subplots)
figure(2);
clf; % Clear the figure before plotting new data

% Extract alarm data
delta_s = logsout.get('delta_s').Values.Data;
lambda_c = logsout.get('S_out').Values.Data;
lambda_r = logsout.get('r').Values.Data;

% Get time vector (assuming the alarms share the same time reference)
time = logsout.get('r').Values.Time;

% First Subplot - delta_s
subplot(3,1,1); % 2 rows, 1 column, first subplot
plot(time, delta_s, 'g', 'LineWidth', 1.5);
ylabel('||\Delta v_s||');
title('Attack Stealthiness Statistics \Delta v_s');
grid on;
if ~isnan(attack_success_time)
    xline(attack_success_time, '--k', 'T_{AS}', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
end

% Second Subplot - c_alarm
subplot(3,1,2); % 2 rows, 1 column, first subplot
plot(time, lambda_c, 'r', 'LineWidth', 1.5);
title('CUSUM Statistics');
ylabel('Anomaly Score');
grid on;
if ~isnan(attack_success_time)
    xline(attack_success_time, '--k', 'T_{AS}', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
end

% Third Subplot - r_alarm
subplot(3,1,3); % 2 rows, 1 column, second subplot
plot(time, lambda_r, 'b', 'LineWidth', 1.5);
title('\chi^2 Statistics');
xlabel('Time (s)');
ylabel('Anomaly Score');
grid on;
if ~isnan(attack_success_time)
    xline(attack_success_time, '--k', 'T_{AS}', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
end


%% Spoofing Velocity Plot (Norm and Angle)
figure(3);
clf; % Clear the figure before plotting new data

% Extract spoofing velocity components
spoofing_vx = squeeze(logsout.get('new_s_vx').Values.Data);
spoofing_vy = squeeze(logsout.get('new_s_vy').Values.Data); % Corrected to get Vy

% Compute Norm (Magnitude) and Angle (Direction)
spoofing_v_norm = sqrt(spoofing_vx.^2 + spoofing_vy.^2);
spoofing_v_angle = atan2(spoofing_vy, spoofing_vx); % Angle in radians

% Time vector (assuming it is the same as logsout timestamps)
time = logsout.get('new_s_vx').Values.Time;

% First Subplot - Norm of Spoofing Velocity
subplot(3,1,1); % 3 rows, 1 column, first subplot
plot(time, spoofing_v_norm, 'g', 'LineWidth', 1.5);
ylabel('||v|| (m/s)');
title('Spoofing Velocity Magnitude');
grid on;
if ~isnan(attack_success_time)
    xline(attack_success_time, '--k', 'T_{AS}', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
end

% Second Subplot - Spoofing Angle Over Time
subplot(3,1,2); % 3 rows, 1 column, third subplot
plot(time, rad2deg(spoofing_v_angle), 'm', 'LineWidth', 1.5); % Convert radians to degrees
xlabel('Time (s)');
ylabel('Angle (°)');
title('Spoofing Velocity Angle Over Time');
grid on;
if ~isnan(attack_success_time)
    xline(attack_success_time, '--k', 'T_{AS}', 'LineWidth', 1.5, 'LabelVerticalAlignment', 'top');
end

% Third Subplot - Angle of Spoofing Velocity in Polar Coordinates
subplot(3,1,3); % 3 rows, 1 column, second subplot
if ~isnan(attack_success_time)
    polarplot(spoofing_v_angle(1:idx_attack_success), spoofing_v_norm(1:idx_attack_success), 'mo', 'MarkerSize', 5, 'LineWidth', 1.5);
else
    polarplot(spoofing_v_angle, spoofing_v_norm, 'mo', 'MarkerSize', 5, 'LineWidth', 1.5);
end
title('Spoofing Velocity Direction (Before Attack Success) (Polar)');



