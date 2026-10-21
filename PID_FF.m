%% ME568 - Harsh's Weeks 4-7 Deliverables Verification (Combined Stream)
clear; clc; close all;

fprintf('====================================================\n');
fprintf('   ME568 ART Project: Harsh Tasks Wks 4-7 Test      \n');
fprintf('====================================================\n\n');

%% 1. Structural & Dynamic Parameters
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

tire.Fz0  = 4000;       tire.muy0 = 1.0; tire.pDy2 = -0.08;
tire.a3   = 80000;      tire.a4   = 4000; tire.Cy   = 1.30; tire.Ey = -1.0;
tire.mux0 = 1.0;        tire.pDx2 = -0.08; tire.CKx = 15.0;
tire.Cx   = 1.65;       tire.Ex   = -0.5;  tire.Cxa  = 1.0;  tire.Bxa = 8.0;
tire.Cyk  = 1.0;        tire.Byk  = 8.0;

act.wn       = 2 * pi * 15; % Natural frequency [rad/s]
act.zeta     = 0.707;       % Damping ratio
act.tau      = 0.010;       % Transport delay [s]
act.rate_lim = deg2rad(10); % Rate limit [rad/s]
act.pos_lim  = deg2rad(5);  % Saturation limit [rad]

%% 2. Simulation Environment
Vx_test = 25.0; % Test velocity [m/s]
dt      = 0.002;
t_vec   = 0:dt:3.5;
N       = length(t_vec);

% Step Steer Input: 3 deg front wheel angle at t = 0.5s
delta_f_vec = zeros(1, N);
delta_f_vec(t_vec >= 0.5) = deg2rad(3);

w0 = Vx_test / veh.Rw;
x0 = [Vx_test; 0; 0; w0*ones(4,1); 0; 0; 0];

% Cases to evaluate for Harsh's Tasks:
% Case 1: Passive Baseline (No ART)
% Case 2: Harsh Wks 4-5 Baseline PID (No FF, No GS, No AW)
% Case 3: Harsh Wks 6-7 Feedforward Only (FF-Only, No Feedback)
test_cases(1).name = 'Passive (No ART)';
test_cases(1).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);
test_cases(1).art_active = false;

test_cases(2).name = 'Baseline PID';
test_cases(2).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);
test_cases(2).art_active = true;

test_cases(3).name = 'Feedforward-Only';
test_cases(3).flags = struct('FF_on', 1, 'GS_on', 0, 'AW_on', 0);
test_cases(3).art_active = true;

results = struct();
[Vx_g, df_g, AY_des_mat] = generate_ay_des_surface();

%% 3. Execution Loop
for c = 1:length(test_cases)
    fprintf(' Running Case %d: %s...\n', c, test_cases(c).name);
    
    x_curr     = x0;
    ctrl_state = struct('xi', 0, 'e_prev', 0);
    ay_act     = 0;
    
    % Reset persistent actuator state buffer for each test case
    simulate_actuator([], [], [], true);
    
    ay_hist   = zeros(1, N);
    ay_des_h  = zeros(1, N);
    dr_cmd_h  = zeros(1, N);
    
    for i = 1:N
        t  = t_vec(i);
        df = delta_f_vec(i);
        Vx = x_curr(1);
        
        % 1. Target Ay lookup
        ay_des = interp2(Vx_g, df_g, AY_des_mat, Vx, abs(df), 'linear', 0) * sign(df);
        
        % 2. Evaluate Rear Steer Command
        if test_cases(c).art_active
            if c == 3 % FF-Only: Pass ay_act = ay_des to zero out feedback tracking error
                flags_ff = struct('FF_on', 1, 'GS_on', 0, 'AW_on', 0);
                [dr_cmd, ctrl_state] = art_controller(ay_des, ay_des, Vx, df, dt, ...
                    ctrl_state, flags_ff, veh);
            else
                [dr_cmd, ctrl_state] = art_controller(ay_des, ay_act, Vx, df, dt, ...
                    ctrl_state, test_cases(c).flags, veh);
            end
        else
            dr_cmd = 0;
        end
        
        % 3. Actuator dynamics & Ackermann splitter
        [dr_act, ~]  = simulate_actuator([0 dt], [dr_cmd dr_cmd], act);
        [d_rl, d_rr] = ackermann_rear_splitter(df, dr_act(end), veh.L, veh.t);
        
        % 4. Advance Plant & Measure Actual Output
        u_plant = [df; df; d_rl; d_rr; zeros(4,1)];
        [dx_plant, plant_out] = vehicle_7dof(t, x_curr, u_plant, veh, tire);
        
        % Read actual lateral acceleration output from plant
        ay_act = plant_out.ay;
        
        % State integration
        x_curr = x_curr + dx_plant * dt;
        
        % Store telemetry
        ay_hist(i)  = ay_act;
        ay_des_h(i) = ay_des;
        dr_cmd_h(i) = dr_cmd;
    end
    
    results(c).ay     = ay_hist;
    results(c).ay_des = ay_des_h;
    results(c).dr_cmd = dr_cmd_h;
end

%% 4. Plot Verification Visuals
figure('Name', 'Harsh Tasks Wks 4-7 Verification');
subplot(2,1,1);
plot(t_vec, results(1).ay_des, 'k:', 'LineWidth', 1.5); hold on;
plot(t_vec, results(1).ay, 'g-.', 'LineWidth', 1.5);
plot(t_vec, results(2).ay, 'r--', 'LineWidth', 1.8);
plot(t_vec, results(3).ay, 'b-', 'LineWidth', 2.0);
grid on; ylabel('Lateral Accel a_y [m/s^2]');
title('Verification: Step Steer Comparison at 25 m/s');
legend('Target a_{y,des}', 'Passive (No ART)', 'Baseline PID (W4-5)', 'FF-Only (W6-7)');

subplot(2,1,2);
plot(t_vec, rad2deg(results(1).dr_cmd), 'g-.', 'LineWidth', 1.5); hold on;
plot(t_vec, rad2deg(results(2).dr_cmd), 'r--', 'LineWidth', 1.8);
plot(t_vec, rad2deg(results(3).dr_cmd), 'b-', 'LineWidth', 2.0);
grid on; xlabel('Time [s]'); ylabel('Rear Steer Cmd \delta_r [deg]');
legend('Passive (0^\circ)', 'Baseline PID', 'FF-Only');

fprintf(' -> Verification Completed Successfully!\n\n');