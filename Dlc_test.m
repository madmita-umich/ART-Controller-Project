%% ME568 - Week 8 Phase 2: ISO 3888-2 Double Lane Change (DLC) Test Suite
clear; clc; close all;

fprintf('====================================================\n');
fprintf('   ME568 ART Project: Week 8 ISO 3888-2 DLC Suite   \n');
fprintf('====================================================\n\n');

%% 1. Parameter Struct Definitions
veh.m     = 1500;       veh.g     = 9.81;       veh.a     = 1.15;
veh.b     = 1.55;       veh.L     = veh.a + veh.b; veh.h  = 0.50;
veh.t     = 1.55;       veh.Iz    = 2250;       veh.Iw    = 1.2;
veh.Rw    = 0.32;       veh.rho   = 1.225;      veh.Af    = 2.2;
veh.Cd    = 0.30;       veh.Cl    = 0.10;       veh.mu_rr = 0.015;
veh.c_brg = 0.05;

tire.Fz0  = 4000;       tire.muy0 = 1.0;        tire.pDy2 = -0.08;
tire.a3   = 80000;      tire.a4   = 4000;       tire.Cy   = 1.30; tire.Ey = -1.0;
tire.mux0 = 1.0;        tire.pDx2 = -0.08;      tire.CKx  = 15.0;
tire.Cx   = 1.65;       tire.Ex   = -0.5;       tire.Cxa  = 1.0;  tire.Bxa = 8.0;
tire.Cyk  = 1.0;        tire.Byk  = 8.0;

act.wn       = 2 * pi * 15;
act.zeta     = 0.707;
act.tau      = 0.010;
act.rate_lim = deg2rad(10);
act.pos_lim  = deg2rad(5);

%% 2. ISO 3888-2 Trajectory & Test Setup
[ref_X, ref_Y] = generate_iso3888_2_path(0.05);

Vx_entry = 22.2; % Entry speed: 80 km/h (22.2 m/s)
dt       = 0.002;
t_final  = 6.0;
t_vec    = 0:dt:t_final;
N        = length(t_vec);

w0 = Vx_entry / veh.Rw;
% State vector: [Vx, Vy, r, w1..4, X, Y, psi]
x0 = [Vx_entry; 0; 0; w0*ones(4,1); 0; 0; 0];

% Evaluation Cases
test_cases(1).name = 'Passive (No ART)';
test_cases(1).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);
test_cases(1).art_active = false;

test_cases(2).name = 'Baseline PI';
test_cases(2).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);
test_cases(2).art_active = true;

test_cases(3).name = 'Full (FF + GS + AW)';
test_cases(3).flags = struct('FF_on', 1, 'GS_on', 1, 'AW_on', 1);
test_cases(3).art_active = true;

results = struct();
[Vx_g, df_g, AY_des_mat] = generate_ay_des_surface();

%% 3. Closed-Loop Execution Loop
for c = 1:length(test_cases)
    fprintf(' Running Case %d: %s...\n', c, test_cases(c).name);
    
    x_curr     = x0;
    ctrl_state = struct('xi', 0, 'e_prev', 0);
    ay_act     = 0;
    
    simulate_actuator([], [], [], true); % Reset actuator persistent state
    
    X_hist      = zeros(1, N);
    Y_hist      = zeros(1, N);
    psi_hist    = zeros(1, N);
    ay_hist     = zeros(1, N);
    df_hist     = zeros(1, N);
    dr_cmd_hist = zeros(1, N);
    dr_act_hist = zeros(1, N);
    
    for i = 1:N
        t  = t_vec(i);
        Vx = x_curr(1);
        Vy = x_curr(2);
        r  = x_curr(3);
        X  = x_curr(8);
        Y  = x_curr(9);
        psi= x_curr(10);
        
        % 1. Closed-Loop Pure Pursuit Driver Model
        [df, ~] = pure_pursuit_driver(X, Y, psi, Vx, ref_X, ref_Y, veh);
        
        % 2. Target Lateral Acceleration Lookup
        ay_des = interp2(Vx_g, df_g, AY_des_mat, Vx, abs(df), 'linear', 0) * sign(df);
        
        % 3. ART Controller Evaluation
        if test_cases(c).art_active
            [dr_cmd, ctrl_state] = art_controller(ay_des, ay_act, Vx, df, dt, ...
                ctrl_state, test_cases(c).flags, veh);
        else
            dr_cmd = 0;
        end
        
        % 4. Actuator Dynamics & Ackermann Rear Splitter
        [dr_act, ~]  = simulate_actuator([0 dt], [dr_cmd dr_cmd], act);
        [d_rl, d_rr] = ackermann_rear_splitter(df, dr_act(end), veh.L, veh.t);
        
        % 5. Advance 7-DOF Vehicle Plant
        u_plant = [df; df; d_rl; d_rr; zeros(4,1)];
        [dx_plant, plant_out] = vehicle_7dof(t, x_curr, u_plant, veh, tire);
        
        ay_act = plant_out.ay;
        x_curr = x_curr + dx_plant * dt;
        
        % Telemetry Storage
        X_hist(i)      = X;
        Y_hist(i)      = Y;
        psi_hist(i)    = psi;
        ay_hist(i)     = ay_act;
        df_hist(i)     = df;
        dr_cmd_hist(i) = dr_cmd;
        dr_act_hist(i) = dr_act(end);
    end
    
    results(c).X      = X_hist;
    results(c).Y      = Y_hist;
    results(c).psi    = psi_hist;
    results(c).ay     = ay_hist;
    results(c).df     = df_hist;
    results(c).dr_cmd = dr_cmd_hist;
    results(c).dr_act = dr_act_hist;
end

%% 4. Plot ISO 3888-2 DLC Verification Results
figure('Name', 'Week 8 ISO 3888-2 Trajectory Tracking');
subplot(2,1,1);
plot(ref_X, ref_Y, 'k--', 'LineWidth', 1.8); hold on;
plot(results(1).X, results(1).Y, 'g-.', 'LineWidth', 1.5);
plot(results(2).X, results(2).Y, 'r:', 'LineWidth', 1.8);
plot(results(3).X, results(3).Y, 'b-', 'LineWidth', 2.0);
grid on; xlabel('Global Position X [m]'); ylabel('Global Position Y [m]');
title('ISO 3888-2 Severe Double Lane Change Trajectory (80 km/h)');
legend('ISO Reference Path', 'Passive (No ART)', 'Baseline PI', 'Full (FF + GS + AW)');

subplot(2,1,2);
plot(t_vec, results(1).ay, 'g-.', 'LineWidth', 1.5); hold on;
plot(t_vec, results(2).ay, 'r:', 'LineWidth', 1.8);
plot(t_vec, results(3).ay, 'b-', 'LineWidth', 2.0');
grid on; xlabel('Time [s]'); ylabel('Lateral Accel a_y [m/s^2]');
title('Vehicle Lateral Acceleration Dynamics');
legend('Passive (No ART)', 'Baseline PI', 'Full (FF + GS + AW)');

figure('Name', 'Week 8 Steering Action Comparison');
subplot(2,1,1);
plot(t_vec, rad2deg(results(1).df), 'g-.', 'LineWidth', 1.5); hold on;
plot(t_vec, rad2deg(results(2).df), 'r:', 'LineWidth', 1.8);
plot(t_vec, rad2deg(results(3).df), 'b-', 'LineWidth', 2.0);
grid on; ylabel('Front Steer \delta_f [deg]');
title('Driver Front Steering Command (Pure Pursuit)');
legend('Passive (No ART)', 'Baseline PI', 'Full (FF + GS + AW)');

subplot(2,1,2);
plot(t_vec, rad2deg(results(1).dr_act), 'g-.', 'LineWidth', 1.5); hold on;
plot(t_vec, rad2deg(results(2).dr_act), 'r:', 'LineWidth', 1.8);
plot(t_vec, rad2deg(results(3).dr_act), 'b-', 'LineWidth', 2.0);
grid on; xlabel('Time [s]'); ylabel('Actual Rear Steer \delta_r [deg]');
title('ART Rear Steering System Action');
legend('Passive (0^\circ)', 'Baseline PI', 'Full (FF + GS + AW)');

fprintf(' -> Week 8 ISO 3888-2 DLC Test Suite Executed Successfully!\n\n');