%% ME568 - Weeks 4-5 Master Execution Script (Madhumita Singh)
clear; clc; close all;

fprintf('====================================================\n');
fprintf('   ME568 ART Project: Weeks 4-5 Pure MATLAB Suite   \n');
fprintf('====================================================\n\n');

%% 1. Generate Target Surface & Test Interpolation
[Vx_grid, delta_f_grid, AY_des] = generate_ay_des_surface();

figure('Name', 'Target Ay Surface');
[VX, DELTA_F] = meshgrid(Vx_grid, delta_f_grid);
surf(VX, rad2deg(DELTA_F), AY_des);
colorbar; grid on;
xlabel('Speed V_x [m/s]'); ylabel('Front Steer \delta_f [deg]');
zlabel('Desired a_{y,des} [m/s^2]');
title('Target Lateral Acceleration Reference Surface');

% Test Online Lookup Query via interp2
test_Vx = 25.0; % m/s
test_df = deg2rad(5.0); % rad
target_ay = interp2(Vx_grid, delta_f_grid, AY_des, test_Vx, test_df, 'linear');
fprintf(' Lookup Test: Vx = %.1f m/s, df = %.1f deg -> Target Ay = %.2f m/s^2\n', ...
    test_Vx, rad2deg(test_df), target_ay);

%% 2. Verify Pacejka Combined Slip Curves
alpha_sweep = deg2rad(-10:0.2:10);
kappa_vals  = [0.0, 0.1, 0.2, 0.3];
Fz_test     = 4000; % N
mu_test     = 1.0;

figure('Name', 'Pacejka Combined Slip');
colors = {'k-', 'b--', 'r-.', 'g:'};

for k_idx = 1:length(kappa_vals)
    kappa_curr = kappa_vals(k_idx);
    Fy_out = zeros(size(alpha_sweep));
    
    for a_idx = 1:length(alpha_sweep)
        [~, Fy_out(a_idx)] = pacejka96_combined_slip(alpha_sweep(a_idx), kappa_curr, Fz_test, mu_test);
    end
    
    plot(rad2deg(alpha_sweep), Fy_out, colors{k_idx}, 'LineWidth', 1.8); hold on;
end

grid on; xlabel('Slip Angle \alpha [deg]'); ylabel('Lateral Force F_y [N]');
title('Pacejka Combined Slip: Fy vs. Slip Angle across Slip Ratios \kappa');
legend('\kappa = 0%', '\kappa = 10%', '\kappa = 20%', '\kappa = 30%');

fprintf(' -> Weeks 4-5 MATLAB Suite executed successfully!\n\n');