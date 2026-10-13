%% ============================================================
% LOCAL CLOSED-LOOP STABILITY CHECK FOR NN CONTROLLER
% ============================================================

disp('Checking local closed-loop stability of NN controller...');

c1 = 1.035454e-2;
c2 = 1.143e-3;

href_list = [0.05 0.10 0.15 0.20 0.25 0.30];

eps_fd = 1e-5;

fprintf('\n========= LOCAL STABILITY CHECK =========\n');
fprintf('%8s %12s %12s %12s %12s %14s %14s %12s\n', ...
    'href', 'k1', 'k2', 'trace', 'det', 'eig1', 'eig2', 'stable');

for i = 1:length(href_list)

    href = href_list(i);

    % Equilibrium point
    h2_eq = href;
    h1_eq = 2 * href;

    % Skip invalid equilibrium
    if h1_eq > 0.6
        fprintf('href = %.3f is infeasible because h1_eq > 0.6\n', href);
        continue;
    end

    % Linearization constants
    delta_eq = h1_eq - h2_eq;

    a = c1 / (2 * sqrt(delta_eq));
    b = c1 / (2 * sqrt(h2_eq));

    % NN finite-difference derivatives
    u0 = net([h1_eq; h2_eq; href]);

    u_h1_plus  = net([h1_eq + eps_fd; h2_eq; href]);
    u_h1_minus = net([h1_eq - eps_fd; h2_eq; href]);

    u_h2_plus  = net([h1_eq; h2_eq + eps_fd; href]);
    u_h2_minus = net([h1_eq; h2_eq - eps_fd; href]);

    k1 = (u_h1_plus - u_h1_minus) / (2 * eps_fd);
    k2 = (u_h2_plus - u_h2_minus) / (2 * eps_fd);

    % Plant linearization
    A = [-a,  a;
          a, -a-b];

    B = [c2; 0];

    K = [k1, k2];

    Acl = A + B*K;

    trAcl = trace(Acl);
    detAcl = det(Acl);
    eigAcl = eig(Acl);

    isStable = all(real(eigAcl) < 0);

    fprintf('%8.3f %12.4f %12.4f %12.6f %12.6f %14.6f %14.6f %12d\n', ...
        href, k1, k2, trAcl, detAcl, eigAcl(1), eigAcl(2), isStable);
end

fprintf('=========================================\n');