% Number of Simulations
% N = 10;
n_initializations = 50;
N = n_initializations;

% MATLAB Time-related Parameters
Time = 200;
sampleTime = 0.01;
T = Time * (1 / sampleTime) + 1;
numSteps = Time * (1 / sampleTime) + 1;
time = sampleTime * (0:numSteps - 1);
time = time';

% Attacker identification and tracking model
attack_pos_noise_level = 0.5;
attack_vel_noise_level = 0.1;


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
gps_vel_noise = 0.05;

imu_acc_noise = sqrt(0.0012356);
imu_gyro_noise = sqrt(deg2rad(0.025));
imu_mag_noise = sqrt(0.3 / sqrt(50));


% Anomaly detector statistics

% % % Parameters for gps noise 0.5
Gamma = timeseries(2.04, time);
Delta_1 = timeseries(3.4, time);
Delta_2 = timeseries(1.1, time);

% % Parameters for gps noise 2.5
% Gamma = timeseries(10.2, time);
% Delta_1 = timeseries(100.4, time);
% Delta_2 = timeseries(5.1, time);

% Initialize variables to store results
s_norm_values = 0.01:0.01:1.0;
% s_norm_limit = 1.0;

% Number of different norms
n_norms = length(s_norm_values);

c_detection_rates = zeros(n_norms, 1);
r_detection_rates = zeros(n_norms, 1);

rmse_vx_s = zeros(n_norms, 1);
rmse_vy_s = zeros(n_norms, 1);

% Main loop
% Outer loop: Loop over different target velocities
% Inner loop: For one attack target velocity, change directions a lot 
% combinations


% Outer loop
for condition_idx = 1:n_norms
    % Get the current attack norm
    current_norm = s_norm_values(condition_idx);

    % Initialize Alarms
    c_alarm = zeros(T, 1);
    r_alarm = zeros(T, 1);

    c_alarm_accum = zeros(T, 1);
    r_alarm_accum = zeros(T, 1);

    rmse_vx_accum = 0;
    rmse_vy_accum = 0;
    

    % Smooth Trajectory
    % Define the time points
    T_waypoints = [0, 50, 100, 150, 200];

    % Define the waypoints corresponding to the time points
    Waypoints = [0, 0, 10;
                 100, 0, 10;
                 100, 100, 20;
                 0, 100, 20;
                 0, 0, 10]; % Repeat the last waypoint to match the time vector size

    % Create the timeseries object
    WaypointSeries = timeseries(Waypoints, T_waypoints);

    % Inner loop
    for i = 1:n_initializations
        % Randomly generate attack initializations
        angle = 2 * pi * rand; % Random angle
        s_x_0 = current_norm * cos(angle);
        s_y_0 = current_norm * sin(angle);

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

        % Attack series
        s_x_0_t = s_x_0 * time;
        s_x_0_series = timeseries(s_x_0_t, time);

        s_y_0_t = s_y_0 * time;
        s_y_0_series = timeseries(s_y_0_t, time);

        % Run Simulink
        sim('Quad_PID_FUSION.slx');

        % Extract data from simulink

        % Attacker predicted drone pose
        attacker_pos_estimate = logsout.get('attacker_pos_obs').Values.Data;
        attacker_pos_estimate_flattened = reshape(attacker_pos_estimate, 1, []);
        attacker_vel_estimate = logsout.get('attacker_vel_obs').Values.Data;
        attacker_vel_estimate_flattened = reshape(attacker_vel_estimate, 1, []);

        % Actual drone velocities
        vel_x_ground_truth = logsout.get('vel_x').Values.Data;
        vel_y_ground_truth = logsout.get('vel_y').Values.Data;

        % Drone velocities after attack
        attacked_vel_x = logsout.get('vel_x_fused').Values.Data;
        attacked_vel_y = logsout.get('vel_y_fused').Values.Data;

        actual_vel_x_deviation = attacked_vel_x - vel_x_ground_truth;
        actual_vel_y_deviation = attacked_vel_y - vel_y_ground_truth;
        
        % Calculate the squared error for x and y deviations
        squared_error_x = (actual_vel_x_deviation + s_x_0).^2;
        squared_error_y = (actual_vel_y_deviation + s_y_0).^2;
        mean_squared_error_x = mean(squared_error_x);
        mean_squared_error_y = mean(squared_error_y);
        rmse_vx = sqrt(mean_squared_error_x);
        rmse_vy = sqrt(mean_squared_error_y);

        % fprintf('RMSE for x-deviation: %.4f\n', rmse_vx);
        % fprintf('RMSE for y-deviation: %.4f\n', rmse_vy);

        rmse_vx_accum = rmse_vx_accum + sqrt(mean_squared_error_x);
        rmse_vy_accum = rmse_vy_accum + sqrt(mean_squared_error_y);

        % Anomaly statistics 
        lambda_c = logsout.get('S_out').Values.Data;
        lambda_r = logsout.get('r').Values.Data;

        c_alarm = logsout.get('c_alarm').Values.Data;
        r_alarm = logsout.get('r_alarm').Values.Data;

        c_alarm_accum = c_alarm_accum + logsout.get('c_alarm').Values.Data;
        r_alarm_accum = r_alarm_accum + logsout.get('r_alarm').Values.Data;

        % Calculate statistics
        c_rate = sum(c_alarm) / T * 100;
        r_rate = sum(r_alarm) / T * 100;

    end

    % Compute averages
    c_alarm_average = c_alarm_accum / N;
    r_alarm_average = r_alarm_accum / N;

    % Compute detection rates
    c_detection_rates(condition_idx) = sum(c_alarm_average) / 20000 * 100;
    r_detection_rates(condition_idx) = sum(r_alarm_average) / 20000 * 100;

    % 
    rmse_vx_average = rmse_vx_accum / N;
    rmse_vy_average = rmse_vy_accum / N;

    rmse_vx_s(condition_idx) = rmse_vx_average;
    rmse_vy_s(condition_idx) = rmse_vy_average;


    % Save results for the current condition
    fprintf('Condition %d/%d: s_norm = %.3f, c_rate = %.3f%%, r_rate = %.3f%%, rmse_vx = %.3f, rmse_vy = %.3f\n', ...
            condition_idx, n_norms, s_norm_values(condition_idx), c_detection_rates(condition_idx), r_detection_rates(condition_idx), rmse_vx_s(condition_idx), rmse_vy_s(condition_idx));
end

% Save aggregated results to CSV
aggregated_results_table = table(s_norm_values', c_detection_rates, r_detection_rates, rmse_vx_s, rmse_vy_s, ...
    'VariableNames', {'s_norm_values', 'c_detection_rate', 'r_detection_rate', 'rmse_vx', 'rmse_vy'});
writetable(aggregated_results_table, 'detection_rates_and_attack_result_rtk_gps_s100_n50_011225_01.csv');


% Save detailed results ro MAT
% Convert structure to table for saving as CSV
% detailed_table = struct2table(detailed_results);
% writetable(detailed_table, 'test_result_complete_dataset_01.csv');
% save('detailed_results_rtk_gps_100_010901.mat', 'detailed_results', '-v7.3');

