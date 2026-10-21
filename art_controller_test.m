%% ME568 - Weeks 6-7 Master Execution Script (Madhumita Singh)
clear; clc; close all;

fprintf('====================================================\n');
fprintf('   ME568 ART Project: Weeks 6-7 Closed-Loop Test   \n');
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

%% 2. Simulation Setup
Vx_test = 25.0; % 25 m/s (~90 km/h)
dt      = 0.002;
t_vec   = 0:dt:4.0;
N       = length(t_vec);

delta_f_vec = zeros(1, N);
delta_f_vec(t_vec >= 0.5) = deg2rad(3); % 3 deg step steer at t = 0.5s

w0 = Vx_test / veh.Rw;
x0 = [Vx_test; 0; 0; w0*ones(4,1); 0; 0; 0];

test_cases(1).name = 'Baseline PI';
test_cases(1).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);

test_cases(2).name = 'Full (FF + GS + AW)';
test_cases(2).flags = struct('FF_on', 1, 'GS_on', 1, 'AW_on', 1);

results = struct();
[Vx_g, df_g, AY_des_mat] = generate_ay_des_surface();

%% 3. Closed-Loop Simulation Loop
for c = 1:length(test_cases)
    fprintf(' Simulating Case %d: %s...\n', c, test_cases(c).name);
    
    x_curr     = x0;
    ctrl_state = struct('xi', 0, 'e_prev', 0);
    ay_act     = 0;
    
    ay_hist   = zeros(1, N);
    ay_des_h  = zeros(1, N);
    dr_cmd_h  = zeros(1, N);
    
    for i = 1:N
        t  = t_vec(i);
        df = delta_f_vec(i);
        Vx = x_curr(1);
        
        % 1. Target Ay lookup query
        ay_des = interp2(Vx_g, df_g, AY_des_mat, Vx, abs(df), 'linear', 0) * sign(df);
        
        % 2. Evaluate Closed-Loop ART Controller
        [dr_cmd, ctrl_state] = art_controller(ay_des, ay_act, Vx, df, dt, ...
            ctrl_state, test_cases(c).flags, veh);
        
        % 3. Actuator Dynamics & Ackermann Splitter
        [dr_act, ~] = simulate_actuator([0 dt], [dr_cmd dr_cmd], act);
        [d_rl, d_rr] = ackermann_rear_splitter(df, dr_act(end), veh.L, veh.t);
        
        % 4. Advance Vehicle Plant ODE with Control Vector u_plant
        u_plant = [df; df; d_rl; d_rr; zeros(4,1)];
        [dx_plant, plant_out] = vehicle_7dof(t, x_curr, u_plant, veh, tire);
        
        % Read actual lateral acceleration output from plant
        ay_act = plant_out.ay;
        x_curr = x_curr + dx_plant * dt;
        
        % Telemetry storage
        ay_hist(i)  = ay_act;
        ay_des_h(i) = ay_des;
        dr_cmd_h(i) = dr_cmd;
    end
    
    results(c).t      = t_vec;
    results(c).ay     = ay_hist;
    results(c).ay_des = ay_des_h;
    results(c).dr_cmd = dr_cmd_h;
end

%% 4. Plot Comparative Results
figure('Name', 'Weeks 6-7 Controller Verification');
subplot(2,1,1);
plot(t_vec, results(1).ay_des, 'k:', 'LineWidth', 1.5); hold on;
plot(t_vec, results(1).ay, 'r--', 'LineWidth', 1.8);
plot(t_vec, results(2).ay, 'b-', 'LineWidth', 2.0);
grid on; ylabel('Lateral Accel a_y [m/s^2]');
title(sprintf('Closed-Loop Tracking Comparison at V_x = %.0f m/s', Vx_test));
legend('Target a_{y,des}', 'Baseline PI', 'Full (FF + GS + AW)');

subplot(2,1,2);
plot(t_vec, rad2deg(results(1).dr_cmd), 'r--', 'LineWidth', 1.8); hold on;
plot(t_vec, rad2deg(results(2).dr_cmd), 'b-', 'LineWidth', 2.0);
grid on; xlabel('Time [s]'); ylabel('Rear Steer Command \delta_r [deg]');
legend('Baseline PI', 'Full (FF + GS + AW)');

fprintf(' -> Weeks 6-7 Closed-Loop Verification Executed Successfully!\n\n');