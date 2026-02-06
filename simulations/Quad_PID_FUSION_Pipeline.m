% Number of Simulations
N = 10;

% Time-related Parameters
Time = 200;
sampleTime = 0.01;
T = Time*(1/sampleTime)+1;
numSteps = Time*(1/sampleTime)+1;
time = sampleTime*(0:numSteps-1);
time = time';

% Attacker prediction model
attack_pos_noise_level = 0.5;
attack_vel_noise_level = 0.1;

attack_pos_noise = attack_pos_noise_level * randn(numSteps, 3); % First 3 fields
attack_vel_noise = attack_vel_noise_level * randn(numSteps, 3); % Last 3 fields
attacker_combined_noise = [attack_pos_noise, attack_vel_noise];

% Create the timeseries object
Attacker_Noise = timeseries(attacker_combined_noise, time);


% System parameters
quad_model;
Quad_Model = struct(...
    'Ixx', 0.016, ...
    'Iyy', 0.016, ...
    'Izz', 0.0274, ...
    'Im', 3.789e-6, ...
    'mass', 1, ...
    'g', 9.8);

% Initialize Alarms
c_alarm = zeros(T,1);
r_alarm = zeros(T,1);

% AD statistics
% Parameters for gps noise 0.1
% Gamma = timeseries(0.405, time);
% Delta_1 = timeseries(0.165, time);
% Delta_2 = timeseries(0.0001, time);

% % % Parameters for gps noise 0.2
% Gamma = timeseries(0.815, time);
% Delta_1 = timeseries(0.66, time);
% Delta_2 = timeseries(0.0001, time);

% Parameters for gps noise 0.22
% Gamma = timeseries(0.9, time);
% Delta_1 = timeseries(0.81, time);
% Delta_2 = timeseries(0.0001, time);

% % Parameters for gps noise 0.5
Gamma = timeseries(2.04, time);
Delta_1 = timeseries(3.4, time);
Delta_2 = timeseries(1.1, time);

% % Parameters for gps noise 2.5
Gamma = timeseries(10.2, time);
Delta_1 = timeseries(100.4, time);
Delta_2 = timeseries(5.1, time);



% Attack series
s_x_0_t = 1 * time;
s_x_0_series = timeseries(s_x_0_t, time);


% Attack initial condition
s_x_0 = 0.0;
s_y_0 = 0.0;
% % 
% s_x_0 = 0.05/sqrt(2);
% s_y_0 = 0.05/sqrt(2);

% Smooth Trajectory
% Define the time points
T = [0, 50, 100, 150, 200];

% Define the waypoints corresponding to the time points
Waypoints = [0, 0, 10;
             100, 0, 10; 
             100, 100, 20; 
             0, 100, 20; 
             0, 0, 10]; % Repeat the last waypoint to match the time vector size

% Create the timeseries object
WaypointSeries = timeseries(Waypoints, T);

% Main loop
for i=1:N
i
% Noise profiles
system_noise = 0.1;

gps_pos_noise = 2.5;
gps_vel_noise = 0.05;

imu_acc_noise = sqrt(0.0012356);
imu_gyro_noise = sqrt(deg2rad(0.025));
imu_mag_noise = sqrt(0.3/ sqrt(50));

% Noise passed to Simulink
GPS_Position_Noise = timeseries(gps_pos_noise*randn(numSteps,3), time);
GPS_Velocity_Noise = timeseries(gps_vel_noise*randn(numSteps,3), time);

IMU_Acc_Noise =  timeseries(imu_acc_noise*randn(numSteps,3), time);
IMU_Gyro_Noise = timeseries(imu_gyro_noise*randn(numSteps,3), time);
IMU_Mag_Noise = timeseries(imu_mag_noise*randn(numSteps,3), time);

System_Noise = timeseries(system_noise*randn(numSteps,12),time);


% Run simulink
sim('Quad_PID_FUSION.slx');
c_alarm = c_alarm + logsout.get('c_alarm').Values.Data;
r_alarm = r_alarm + logsout.get('r_alarm').Values.Data;

end

% Log alarm data
c_alarm_average = c_alarm/N;
save('c_alarm_average','c_alarm_average')
r_alarm_average = r_alarm/N;
save('r_alarm_average','r_alarm_average')

c_alarm_in_window = c_alarm_average(1:20000);
r_alarm_in_window = r_alarm_average(1:20000);

% Plot in matlab
figure(1);
plot(c_alarm_in_window,'LineWidth',1.5,'color','b');

xlabel('Time (s)')
ylabel('Alarm Rate')
legend('Alarm rate for attack with $b_{\zeta}=.05$ using CUSUM ID','Interpreter','latex');
%xlim([-40 40])
ylim([0 1])
grid on 

figure(2);
plot(r_alarm_in_window,'LineWidth',1.5,'color','b');hold on;
xlabel('Time (s)')
ylabel('Alarm Rate')
legend('Alarm rate for attack with $b_{\zeta}=.05$ using $\chi^2$ ID','Interpreter','latex')
%xlim([-40 40])
ylim([0 1])
grid on 

% 3d plot
figure(3);
pos_x_visualize = logsout.get('pos_x').Values.Data;
pos_y_visualize = logsout.get('pos_y').Values.Data;
pos_z_visualize = logsout.get('pos_z').Values.Data;

% Plotting pos_x vs pos_y
plot(pos_x_visualize, pos_y_visualize);
xlabel('Position X'); % Label for X-axis
ylabel('Position Y'); % Label for Y-axis
title('2D Position Plot'); % Title for the plot
grid on; % Turn on the grid for better visualization

% Print detection rates
c_detection_rate = (sum(c_alarm_in_window)/20000)*100
r_detection_rate = (sum(r_alarm_in_window)/20000)*100

% Save to csv
alarm_to_save = table(c_alarm_in_window, r_alarm_in_window);
alarm_to_save.Properties.VariableNames = {'c_alarm', 'r_alarm'};
% writetable(alarm_to_save, 'Datasets/Alarm Datasets/slow_alarms_no_attack_gps_noise_020_1000_1.csv')
% writetable(alarm_to_save, 'Datasets/Alarm Datasets/slow_alarms_attack_s_005_gps_noise_020_1000_1.csv')



