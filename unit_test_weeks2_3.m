%% ME568 - Weeks 2-3 Master Execution Script (Madhumita Singh)
clear; clc; close all;

fprintf('====================================================\n');
fprintf('   ME568 ART Project: Weeks 2-3 Pure MATLAB Suite   \n');
fprintf('====================================================\n\n');

% Vehicle & Actuator Parameters
params.L  = 2.70;       % Wheelbase [m]
params.a  = 1.15;       % Distance CG to front axle [m]
params.b  = 1.55;       % Distance CG to rear axle [m]
params.tr = 1.55;       % Rear track width [m]
params.m  = 1500;       % Mass [kg]
params.Iz = 2250;       % Yaw moment of inertia [kg*m^2]
params.Caf = 80000;     % Front cornering stiffness per axle [N/rad]
params.Car = 80000;     % Rear cornering stiffness per axle [N/rad]

act.wn       = 2 * pi * 15;        % Natural frequency (15 Hz) [rad/s]
act.zeta     = 0.707;              % Damping ratio
act.tau      = 0.010;              % Transport delay [s]
act.rate_lim = deg2rad(10);        % Rate limit [rad/s]
act.pos_lim  = deg2rad(5);         % Saturation limit [rad]

%% 1. Actuator Step Response Simulation
t_sim = 0:0.001:0.3;
step_cmd = deg2rad(3) * ones(size(t_sim)); % 3 degree step command

[delta_r_sim, ~] = simulate_actuator(t_sim, step_cmd, act);

figure('Name', 'Actuator Step Response in MATLAB');
plot(t_sim*1000, rad2deg(step_cmd), 'k:', 'LineWidth', 1.5); hold on;
plot(t_sim*1000, rad2deg(delta_r_sim), 'b-', 'LineWidth', 2.0);
grid on; xlabel('Time [ms]'); ylabel('Rear Steer Angle [deg]');
title('Pure MATLAB Actuator Dynamics Response (10ms Delay + Rate/Sat Limits)');
legend('Step Command', 'Actuator Output');

%% 2. Ackermann Splitter Verification
delta_f_sweep = deg2rad(-15:0.5:15);
delta_r_cmd   = deg2rad(2);

delta_rl_out = zeros(size(delta_f_sweep));
delta_rr_out = zeros(size(delta_f_sweep));

for i = 1:length(delta_f_sweep)
    [delta_rl_out(i), delta_rr_out(i)] = ackermann_rear_splitter(...
        delta_f_sweep(i), delta_r_cmd, params.L, params.tr);
end

figure('Name', 'Ackermann Splitter Output');
plot(rad2deg(delta_f_sweep), rad2deg(delta_rl_out), 'b-o', 'LineWidth', 1.2); hold on;
plot(rad2deg(delta_f_sweep), rad2deg(delta_rr_out), 'r-s', 'LineWidth', 1.2);
grid on; xlabel('Front Steer Angle \delta_f [deg]');
ylabel('Individual Rear Steer Angle [deg]');
title('Ackermann Rear-Toe Splitter Output (\delta_r = 2^\circ)');
legend('Rear-Left (\delta_{rl})', 'Rear-Right (\delta_{rr})');

fprintf(' -> Weeks 2-3 MATLAB Suite executed successfully!\n\n');