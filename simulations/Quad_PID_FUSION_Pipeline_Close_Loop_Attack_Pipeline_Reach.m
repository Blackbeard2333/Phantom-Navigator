% Close-loop GPS Spoofing Attack Pipeline Reachability analysis
% clear;

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

delta_s_value = 2;
delta_s_limit = delta_s_value * ones(length(time), 1);
Delta_S_Lim = timeseries(delta_s_limit, time);

% Define spoofing target
spoofing_target_x = 1000;
spoofing_target_y = 1000;
target_radius_value = 10;

% Create a matching-size array
spoofing_target = [spoofing_target_x * ones(length(time), 1), ...
                   spoofing_target_y * ones(length(time), 1)];
target_radius = target_radius_value * ones(length(time), 1);
Target_R = timeseries(target_radius, time);

% Convert to timeseries
Spoofing_Target = timeseries(spoofing_target, time);


% ======= Define System Dynamics =======
dt = 0.01;  % Time step
T_max = 400;  % Time horizon
num_iterations = T_max / dt;

% State transition matrix (Position, Velocity, Spoofing Velocity)
A = [1, 0, dt, 0, dt, 0;
     0, 1, 0, dt, 0, dt;
     0, 0, 1,  0, 1, 0;
     0, 0, 0,  1, 0, 1;
     0, 0, 0,  0, 1, 0;
     0, 0, 0,  0, 0, 1];

B = [0, 0;
     0, 0;
     0, 0;
     0, 0;
     dt, 0;
     0, dt];

% Control constraints
delta_s_lim = 1; % Maximum spoofing acceleration

% ======= Define Discretized State Space =======
grid_min = [-1000; -1000; -5; -5; -1; -1];  % Lower bounds
grid_max = [1000; 1000; 5; 5; 1; 1];  % Upper bounds
N = [100, 100, 10, 10, 10, 10];  % Number of grid points in each dimension

% Compute grid resolution
dx = (grid_max - grid_min) ./ (N' - 1);

% Generate mesh grid for state variables
[p_x, p_y, v_x, v_y, s_vx, s_vy] = ndgrid(...
    linspace(grid_min(1), grid_max(1), N(1)), ...
    linspace(grid_min(2), grid_max(2), N(2)), ...
    linspace(grid_min(3), grid_max(3), N(3)), ...
    linspace(grid_min(4), grid_max(4), N(4)), ...
    linspace(grid_min(5), grid_max(5), N(5)), ...
    linspace(grid_min(6), grid_max(6), N(6)) );

% Initialize value function V (cost-to-go)
V = inf(size(p_x));  % Start with large cost (unreachable)


% ======= Define Initial Set =======
x0 = [0; 0; 0; 0; 0; 0];  % Initial condition
initial_radius = 1;  % Initial uncertainty region

% Compute distance from initial state
initial_mask = (p_x - x0(1)).^2 + (p_y - x0(2)).^2 <= initial_radius^2;

% Set initial value function: 0 for initial states, inf elsewhere
V(initial_mask) = 0;

% ======= Solve HJB PDE Forward in Time =======
for t = 1:num_iterations
    % Compute spatial gradient using finite differences
    [Vx, Vy, Vvx, Vvy, Vsx, Vsy] = gradient(V, dx(1), dx(2), dx(3), dx(4), dx(5), dx(6));
    
    % Compute optimal control (maximize reachability expansion)
    u_opt_x = sign(Vsx) * delta_s_lim;  % Maximize in all directions
    u_opt_y = sign(Vsy) * delta_s_lim;
    
    % Compute Hamiltonian expansion
    dV_dt = Vx .* v_x + Vy .* v_y + Vvx .* s_vx + Vvy .* s_vy + Vsx .* u_opt_x + Vsy .* u_opt_y;
    
    % Update value function using explicit time stepping
    V = V - dt * max(dV_dt, 0);
end

% ======= Extract Reachable Set =======
reachable_set = (V < inf);  % Identify all states that can be reached

% ======= Compute the Boundary of the Reachable Set =======
% Use a morphological edge detector to find boundary points
reachable_boundary = reachable_set & ~imdilate(reachable_set, ones(3,3,3,3,3,3));

% ======= Plot Reachable Set Boundary =======
figure;
scatter(p_x(reachable_boundary), p_y(reachable_boundary), 'r.');
hold on;
scatter(x0(1), x0(2), 100, 'gx', 'LineWidth', 2);
xlabel('Position X');
ylabel('Position Y');
title('Boundary of the Forward Reachability Set Computed Using Hamilton-Jacobi PDEs');
legend('Reachable Set Boundary', 'Initial Condition');
grid on;

