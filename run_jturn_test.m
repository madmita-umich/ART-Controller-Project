function run_jturn_test()
% RUN_JTURN_TEST Executes Harsh's Week 8 J-Turn high-g rollover/spin test.

    fprintf('--- Executing Harsh Wk 8: J-Turn Limit Handling Test ---\n');

    %% 1. Parameters & Setup
    veh = get_veh_params();
    tire = get_tire_params();
    act = get_act_params();

    Vx_test = 22.2; % 80 km/h entry speed
    dt      = 0.002;
    t_vec   = 0:dt:4.0;
    N       = length(t_vec);

    % Rapid Steering Ramp: 0 to 8 deg in 0.15s starting at t = 0.5s, then held
    delta_f_vec = zeros(1, N);
    for i = 1:N
        if t_vec(i) >= 0.5 && t_vec(i) < 0.65
            delta_f_vec(i) = deg2rad(8) * ((t_vec(i) - 0.5) / 0.15);
        elseif t_vec(i) >= 0.65
            delta_f_vec(i) = deg2rad(8);
        end
    end

    w0 = Vx_test / veh.Rw;
    x0 = [Vx_test; 0; 0; w0*ones(4,1); 0; 0; 0];

    test_cases(1).name = 'Passive (No ART)';
    test_cases(1).art_active = false;
    test_cases(1).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);

    test_cases(2).name = 'Baseline PI';
    test_cases(2).art_active = true;
    test_cases(2).flags = struct('FF_on', 0, 'GS_on', 0, 'AW_on', 0);

    test_cases(3).name = 'Full (FF + GS + AW)';
    test_cases(3).art_active = true;
    test_cases(3).flags = struct('FF_on', 1, 'GS_on', 1, 'AW_on', 1);

    results = struct();
    [Vx_g, df_g, AY_des_mat] = generate_ay_des_surface();

    %% 2. Simulation Loop
    for c = 1:length(test_cases)
        x_curr     = x0;
        ctrl_state = struct('xi', 0, 'e_prev', 0);
        ay_act     = 0;
        
        simulate_actuator([], [], [], true); % Reset persistent actuator state
        
        ay_hist   = zeros(1, N);
        r_hist    = zeros(1, N);
        dr_act_h  = zeros(1, N);
        
        for i = 1:N
            t  = t_vec(i);
            df = delta_f_vec(i);
            Vx = x_curr(1);
            
            ay_des = interp2(Vx_g, df_g, AY_des_mat, Vx, abs(df), 'linear', 0) * sign(df);
            
            if test_cases(c).art_active
                [dr_cmd, ctrl_state] = art_controller(ay_des, ay_act, Vx, df, dt, ...
                    ctrl_state, test_cases(c).flags, veh);
            else
                dr_cmd = 0;
            end
            
            [dr_act, ~]  = simulate_actuator([0 dt], [dr_cmd dr_cmd], act);
            [d_rl, d_rr] = ackermann_rear_splitter(df, dr_act(end), veh.L, veh.t);
            
            u_plant = [df; df; d_rl; d_rr; zeros(4,1)];
            [dx_plant, plant_out] = vehicle_7dof(t, x_curr, u_plant, veh, tire);
            
            ay_act = plant_out.ay;
            x_curr = x_curr + dx_plant * dt;
            
            ay_hist(i)  = ay_act;
            r_hist(i)   = plant_out.r;
            dr_act_h(i) = dr_act(end);
        end
        
        results(c).ay     = ay_hist;
        results(c).r      = r_hist;
        results(c).dr_act = dr_act_h;
    end

    %% 3. Plotting J-Turn Visuals
    figure('Name', 'Harsh Wk 8: J-Turn Maneuver');
    subplot(2,1,1);
    plot(t_vec, results(1).ay, 'g-.', 'LineWidth', 1.5); hold on;
    plot(t_vec, results(2).ay, 'r:', 'LineWidth', 1.8);
    plot(t_vec, results(3).ay, 'b-', 'LineWidth', 2.0);
    grid on; ylabel('Lateral Accel a_y [m/s^2]');
    title('J-Turn Limit Handling Response at 80 km/h (\delta_f = 8^\circ)');
    legend('Passive (No ART)', 'Baseline PI', 'Full (FF + GS + AW)');

    subplot(2,1,2);
    plot(t_vec, rad2deg(results(1).r), 'g-.', 'LineWidth', 1.5); hold on;
    plot(t_vec, rad2deg(results(2).r), 'r:', 'LineWidth', 1.8);
    plot(t_vec, rad2deg(results(3).r), 'b-', 'LineWidth', 2.0);
    grid on; xlabel('Time [s]'); ylabel('Yaw Rate r [deg/s]');
    legend('Passive (No ART)', 'Baseline PI', 'Full (FF + GS + AW)');
end

function veh = get_veh_params()
    veh.m     = 1500;       veh.g     = 9.81;       veh.a     = 1.15;
    veh.b     = 1.55;       veh.L     = veh.a + veh.b; veh.h  = 0.50;
    veh.t     = 1.55;       veh.Iz    = 2250;       veh.Iw    = 1.2;
    veh.Rw    = 0.32;       veh.rho   = 1.225;      veh.Af    = 2.2;
    veh.Cd    = 0.30;       veh.Cl    = 0.10;       veh.mu_rr = 0.015;
    veh.c_brg = 0.05;
end

function tire = get_tire_params()
    tire.Fz0  = 4000;       tire.muy0 = 1.0;        tire.pDy2 = -0.08;
    tire.a3   = 80000;      tire.a4   = 4000;       tire.Cy   = 1.30; tire.Ey = -1.0;
    tire.mux0 = 1.0;        tire.pDx2 = -0.08;      tire.CKx  = 15.0;
    tire.Cx   = 1.65;       tire.Ex   = -0.5;       tire.Cxa  = 1.0;  tire.Bxa = 8.0;
    tire.Cyk  = 1.0;        tire.Byk  = 8.0;
end

function act = get_act_params()
    act.wn       = 2 * pi * 15;
    act.zeta     = 0.707;
    act.tau      = 0.010;
    act.rate_lim = deg2rad(10);
    act.pos_lim  = deg2rad(5);
end