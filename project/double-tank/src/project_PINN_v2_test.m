%% ============================================================
%  NOMINAL STEP RESPONSE TEST FOR NN CONTROLLER
%  No noise, no disturbance
% ============================================================

disp('Running nominal closed-loop step response tests...');

%% Plant parameters
% If these are already defined in training script, no need to redefine.
c1 = 1.035454e-2;
c2 = 1.143e-3;
H_max = 0.6;
V_max = 10;

%% Simulation settings
dt = 0.1;
t_end = 500;
t = 0:dt:t_end;
N = length(t);

%% Step references to test
href_list = [0.07, 0.13, 0.22, 0.27, 0.30];

%% Initial condition
h1_0 = 0.0;
h2_0 = 0.0;

%% Storage for performance results
results = struct();

figure('Name', 'Nominal Step Responses - NN Controller', ...
       'Position', [100, 100, 1000, 750]);

for r = 1:length(href_list)

    h_ref = href_list(r);

    h1_sim = zeros(1, N);
    h2_sim = zeros(1, N);
    u1_sim = zeros(1, N);

    h1_sim(1) = h1_0;
    h2_sim(1) = h2_0;

    for k = 1:(N-1)

        % =====================================================
        % 1) Neural Network Controller
        % Input: [h1; h2; h_ref]
        % Output: u1
        % =====================================================
        u_raw = net([h1_sim(k); h2_sim(k); h_ref]);

        % Saturation due to physical pump limit
        u1_sim(k) = max(0, min(u_raw, V_max));

        % =====================================================
        % 2) Nonlinear Coupled Tank Plant
        % =====================================================
        diff_h = h1_sim(k) - h2_sim(k);
        sign_val = sign(diff_h);
        sqrt_diff = sqrt(abs(diff_h));

        dh1 = -c1 * sign_val * sqrt_diff + c2 * u1_sim(k);
        dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(max(h2_sim(k), 0));

        % Euler integration + physical tank limits
        h1_sim(k+1) = h1_sim(k) + dt * dh1;
        h2_sim(k+1) = h2_sim(k) + dt * dh2;

        h1_sim(k+1) = max(0, min(h1_sim(k+1), H_max));
        h2_sim(k+1) = max(0, min(h2_sim(k+1), H_max));
    end

    u1_sim(end) = u1_sim(end-1);

    %% ========================================================
    % Performance metrics
    % =========================================================
    e = h_ref - h2_sim;

    final_h2 = h2_sim(end);
    final_error = abs(h_ref - final_h2);

    overshoot = max(h2_sim) - h_ref;
    overshoot = max(0, overshoot);

    IAE = trapz(t, abs(e));
    ISE = trapz(t, e.^2);

    max_u = max(u1_sim);
    min_u = min(u1_sim);

    % Rise time: time to reach 90% of reference
    idx_rise = find(h2_sim >= 0.9*h_ref, 1, 'first');

    if isempty(idx_rise)
        rise_time_90 = NaN;
    else
        rise_time_90 = t(idx_rise);
    end

    % Settling time: last time after which response stays within 2%
    tolerance = 0.02 * h_ref;
    outside_idx = find(abs(e) > tolerance);

    if isempty(outside_idx)
        settling_time_2 = 0;
    elseif outside_idx(end) == N
        settling_time_2 = NaN;
    else
        settling_time_2 = t(outside_idx(end) + 1);
    end

    % Store results
    results(r).h_ref = h_ref;
    results(r).final_h2 = final_h2;
    results(r).final_error = final_error;
    results(r).overshoot = overshoot;
    results(r).IAE = IAE;
    results(r).ISE = ISE;
    results(r).rise_time_90 = rise_time_90;
    results(r).settling_time_2 = settling_time_2;
    results(r).max_u = max_u;
    results(r).min_u = min_u;

    %% ========================================================
    % Plots
    % =========================================================
    subplot(2,1,1);
    plot(t, h2_sim, 'LineWidth', 1.8); hold on;

    subplot(2,1,2);
    plot(t, u1_sim, 'LineWidth', 1.5); hold on;
end

%% Plot formatting
subplot(2,1,1);
for r = 1:length(href_list)
    yline(href_list(r), '--', ...
        ['h_{ref} = ', num2str(href_list(r)), ' m'], ...
        'LabelHorizontalAlignment', 'left');
end
title('Nominal Closed-Loop Step Responses of h_2');
xlabel('Time (s)');
ylabel('Tank 2 Level h_2 (m)');
grid on;
legend(arrayfun(@(x) ['h_{ref} = ', num2str(x), ' m'], ...
       href_list, 'UniformOutput', false), ...
       'Location', 'best');

subplot(2,1,2);
title('Control Signal u_1');
xlabel('Time (s)');
ylabel('Pump Voltage u_1 (V)');
ylim([-0.5 10.5]);
grid on;
legend(arrayfun(@(x) ['h_{ref} = ', num2str(x), ' m'], ...
       href_list, 'UniformOutput', false), ...
       'Location', 'best');

%% ============================================================
% Print results table
% ============================================================
fprintf('\n================ NOMINAL STEP TEST RESULTS ================\n');
fprintf('%8s %10s %12s %12s %10s %10s %12s %12s %10s\n', ...
    'href', 'final_h2', 'final_err', 'overshoot', ...
    'IAE', 'ISE', 'rise90', 'settle2', 'max_u');

for r = 1:length(results)
    fprintf('%8.3f %10.4f %12.6f %12.6f %10.4f %10.4f %12.2f %12.2f %10.4f\n', ...
        results(r).h_ref, ...
        results(r).final_h2, ...
        results(r).final_error, ...
        results(r).overshoot, ...
        results(r).IAE, ...
        results(r).ISE, ...
        results(r).rise_time_90, ...
        results(r).settling_time_2, ...
        results(r).max_u);
end

fprintf('===========================================================\n');