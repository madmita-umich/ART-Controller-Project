%% ME568 - Master Integrated Vehicle Dynamics Test Harness
clear; clc; close all;

fprintf('====================================================\n');
fprintf('   ME568 ART Project: Integrated System Simulation  \n');
fprintf('====================================================\n\n');

%% 1. Parameter Struct Definitions
% Vehicle Structural Parameters
veh.m     = 1500;       % Total mass [kg]
veh.g     = 9.81;       % Gravity [m/s^2]
veh.a     = 1.15;       % CG to front axle [m]
veh.b     = 1.55;       % CG to rear axle [m]
veh.L     = veh.a + veh.b; % Wheelbase [m]
veh.h     = 0.50;       % CG height [m]
veh.t     = 1.55;       % Track width [m]
veh.Iz    = 2250;       % Yaw moment of inertia [kg*m^2]
veh.Iw    = 1.2;        % Wheel rotational inertia [kg*m^2]
veh.Rw    = 0.32;       % Effective wheel radius [m]
veh.rho   = 1.225;      % Air density [kg/m^3]
veh.Af    = 2.2;        % Frontal area [m^2]
veh.Cd    = 0.30;       % Drag coefficient
veh.Cl    = 0.10;       % Lift coefficient
veh.mu_rr = 0.015;      % Rolling resistance coefficient
veh.c_brg = 0.05;       % Bearing damping coefficient

% Pacejka Tire Parameters
tire.Fz0  = 4000;       % Nominal load [N]
tire.muy0 = 1.0;        % Peak lateral friction
tire.pDy2 = -0.08;      % Lateral load sensitivity
tire.a3   = 80000;      % Cornering stiffness scaling
tire.a4   = 4000;       % Cornering stiffness load factor
tire.Cy   = 1.30; tire.Ey = -1.0;

tire.mux0 = 1.0;        % Peak longitudinal friction
tire.pDx2 = -0.08;      % Longitudinal load sensitivity
tire.CKx  = 15.0;       % Longitudinal slip stiffness factor
tire.Cx   = 1.65; tire.Ex = -0.5;

tire.Cxa  = 1.0; tire.Bxa = 8.0; % Combined slip factors
tire.Cyk  = 1.0; tire.Byk = 8.0;

% Actuator Constraints (Madhumita's Module)
act.wn       = 2 * pi * 15; % Natural frequency (15 Hz) [rad/s]
act.zeta     = 0.707;       % Damping ratio
act.tau      = 0.010;       % Transport delay [s]
act.rate_lim = deg2rad(10); % Rate limit [rad/s]
act.pos_lim  = deg2rad(5);  % Saturation limit [rad]

%% 2. Simulation Test Setup
V0 = 20; % Vehicle speed = 20 m/s (~72 km/h)
tspan = [0 5]; % 5 second simulation run

% Initial States: [Vx, Vy, r, w_fl, w_fr, w_rl, w_rr, X, Y, psi]
w0 = V0 / veh.Rw;
x0 = [V0; 0; 0; w0*ones(4,1); 0; 0; 0];

% Command Inputs: Step steer at t = 0.5s
% Front steer delta_f = 3 deg, Central rear steer command delta_r_cmd = 1 deg
t_vec = tspan(1):0.002:tspan(2);
delta_f_vec     = zeros(size(t_vec));
delta_r_cmd_vec = zeros(size(t_vec));

step_idx = t_vec >= 0.5;
delta_f_vec(step_idx)     = deg2rad(3);
delta_r_cmd_vec(step_idx) = deg2rad(1);

% Pass central rear steer command through Actuator Model
[delta_r_act, ~] = simulate_actuator(t_vec, delta_r_cmd_vec, act);

%% 3. ODE45 Dynamic Integration Loop
fprintf(' Running integrated 7-DOF vehicle dynamic simulation...\n');

% Wrapper function interpolating actuator & steer commands during ODE integration
ode_fun = @(t, x) vehicle_7dof_wrapper(t, x, t_vec, delta_f_vec, delta_r_act, veh, tire);

[t_out, x_out] = ode45(ode_fun, tspan, x0);

%% 4. Post-Processing & Output Generation
N_pts = length(t_out);
ay_vec   = zeros(N_pts, 1);
beta_vec = zeros(N_pts, 1);

for i = 1:N_pts
    [~, out] = ode_fun(t_out(i), x_out(i,:)');
    ay_vec(i)   = out.ay;
    beta_vec(i) = out.beta;
end

% Plot System Responses
figure('Name', 'Integrated Vehicle Response');
subplot(3,1,1);
plot(t_out, rad2deg(x_out(:,3)), 'b-', 'LineWidth', 1.8);
grid on; ylabel('Yaw Rate r [deg/s]');
title('Integrated 7-DOF Vehicle Response (Step Steer + Active Rear Toe)');

subplot(3,1,2);
plot(t_out, ay_vec / veh.g, 'r-', 'LineWidth', 1.8);
grid on; ylabel('Lateral Accel a_y [g]');

subplot(3,1,3);
plot(t_out, rad2deg(beta_vec), 'k-', 'LineWidth', 1.8);
grid on; xlabel('Time [s]'); ylabel('Sideslip Angle \beta [deg]');

fprintf(' -> Integrated Simulation Completed Successfully!\n\n');

%% Helper Wrapper Function
function [dx, out] = vehicle_7dof_wrapper(t, x, t_vec, delta_f_vec, delta_r_vec, veh, tire)
    % Interpolate steer commands at current time step
    df_curr = interp1(t_vec, delta_f_vec, t, 'previous', 'extrap');
    dr_curr = interp1(t_vec, delta_r_vec, t, 'previous', 'extrap');
    
    % Compute Ackermann rear steer angles
    [delta_rl, delta_rr] = ackermann_rear_splitter(df_curr, dr_curr, veh.L, veh.t);
    
    % Build 8x1 control vector: u = [fl fr rl rr Tw_fl Tw_fr Tw_rl Tw_rr]
    u = [df_curr; df_curr; delta_rl; delta_rr; zeros(4,1)];
    
    % Evaluate plant derivative
    if nargout > 1
        [dx, out] = vehicle_7dof(t, x, u, veh, tire);
    else
        dx = vehicle_7dof(t, x, u, veh, tire);
    end
end