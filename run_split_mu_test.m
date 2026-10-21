function run_split_mu_test()
% RUN_SPLIT_MU_TEST Executes Harsh's Week 8 Split-Friction Disturbance Test.

    fprintf('--- Executing Harsh Wk 8: Split-Mu Step Disturbance Test ---\n');

    %% 1. Parameters & Setup
    veh = get_veh_params();
    tire = get_tire_params();
    act = get_act_params();

    % Define Asymmetric Friction Profile across tires [FL, FR, RL, RR]
    tire_split = tire;
    tire_split.muy0 = [0.9, 0.3, 0.9, 0.3]; % High mu left, low mu right
    tire_split.mux0 = [0.9, 0.3, 0.9, 0.3];

    Vx_test = 20.0; % 72 km/h
    dt      = 0.002;
    t_vec   = 0:dt:4.0;
    N       = length(t_vec);

    % Step steer command across asymmetric friction road
    delta_f_vec = zeros(1, N);
    delta_f_vec(t_vec >= 0.5) = deg2rad(3);

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
            
            % Pass tire_split struct containing asymmetric friction
            [dx_plant, plant_out] = vehicle_7dof(t, x_curr, u_plant, veh, tire_split);
            
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

    %% 3. Plotting Split-Mu Visuals
    figure('Name', 'Harsh Wk 8: Split-Mu Disturbance');
    subplot(2,1,1);
    plot(t_vec, results(1).ay, 'g-.', 'LineWidth', 1.5); hold on;
    plot(t_vec, results(2).ay, 'r:', 'LineWidth', 1.8);
    plot(t_vec, results(3).ay, 'b-', 'LineWidth', 2.0);
    grid on; ylabel('Lateral Accel a_y [m/s^2]');
    title('Split-\mu Step Response (\mu_{left} = 0.9, \mu_{right} = 0.3)');
    legend('Passive (No ART)', 'Baseline PI', 'Full (FF + GS + AW)');

    subplot(2,1,2);
    plot(t_vec, rad2deg(results(3).dr_act), 'b-', 'LineWidth', 2.0); hold on;
    plot(t_vec, rad2deg(results(2).dr_act), 'r:', 'LineWidth', 1.8);
    grid on; xlabel('Time [s]'); ylabel('Rear Steer Angle \delta_r [deg]');
    title('ART Controller Intervention under Split-\mu Friction Disturbance');
    legend('Full (FF + GS + AW)', 'Baseline PI');
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