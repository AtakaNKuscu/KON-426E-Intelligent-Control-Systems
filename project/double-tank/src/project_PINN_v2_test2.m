% --- INTELLIGENT CONTROL: EXTREME NOISE REJECTION TEST ---
clear; clc; close all;

%% 1. Eğitilmiş Modeli Yükle
if isfile('trained_pinn_controller.mat')
    load('trained_pinn_controller.mat', 'net');
else
    error('Model dosyası bulunamadı!');
end

%% 2. Sistem Parametreleri ve Gürültü Ayarı
c1 = 1.035454e-2;
c2 = 1.143e-3;    
H_max = 0.6;      
V_max = 10;       

noise_amplitude = 0.015; % +/- 1.5 cm'lik devasa sensör gürültüsü!

%% 3. Test Senaryosu
dt = 0.1;
t_end = 200; 
t = 0:dt:t_end;
N = length(t);

h1_sim = zeros(1, N);
h2_sim = zeros(1, N);
h2_noisy_history = zeros(1, N); % Gürültülü okumayı çizmek için kaydediyoruz
u1_sim = zeros(1, N);
h_ref = 0.25; % Sabit 0.25m hedefi

disp('Agresif Gürültü (Noise) Testi Simüle Ediliyor...');

for k = 1:(N-1)
    % A. SENSÖR OKUMASI (AŞIRI GÜRÜLTÜLÜ)
    % Gerçek su seviyesinin üzerine ciddi bir rastgele gürültü biniyor
    noise1 = noise_amplitude * randn();
    noise2 = noise_amplitude * randn();
    
    h1_read = h1_sim(k) + noise1;
    h2_read = h2_sim(k) + noise2;
    
    h2_noisy_history(k) = h2_read; % Grafikte göstermek için kaydet
    
    % B. NEURAL CONTROLLER KARARI
    % Ağ sadece gürültülü veriyi görüyor
    u1_sim(k) = net([h1_read; h2_read; h_ref]);
    u1_sim(k) = max(0, min(u1_sim(k), V_max)); 
    
    % C. GERÇEK FİZİKSEL SİSTEM (PLANT - PDF Eq 5.9 ve 5.10) [cite: 71, 72]
    diff_h = h1_sim(k) - h2_sim(k);
    sign_val = sign(diff_h);
    sqrt_diff = sqrt(abs(diff_h));
    
    dh1 = -c1 * sign_val * sqrt_diff + c2 * u1_sim(k);
    dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(h2_sim(k));
    
    h1_sim(k+1) = max(0, min(h1_sim(k) + dt * dh1, H_max));
    h2_sim(k+1) = max(0, min(h2_sim(k) + dt * dh2, H_max));
end
u1_sim(end) = u1_sim(end-1);
h2_noisy_history(end) = h2_noisy_history(end-1);

%% 4. Çizimler
figure('Name', 'Extreme Noise Rejection Test', 'Position', [100, 100, 900, 600]);

% Su Seviyeleri (Gerçek vs Gürültülü)
subplot(2,1,1);
plot(t, h2_noisy_history, 'Color', [0.7 0.7 1], 'LineWidth', 1); hold on; % Soluk mavi (Gürültü)
plot(t, h2_sim, 'b', 'LineWidth', 2.5); % Koyu mavi (Gerçek temiz seviye)
plot(t, h_ref * ones(1, N), 'k:', 'LineWidth', 2);
title('Sensor Noise vs. Actual Plant Behavior');
xlabel('Zaman (s)'); ylabel('Su Seviyesi (m)');
legend('Sensörden Okunan (Gürültülü)', 'Gerçekleşen (Filtrelenmiş)', 'Referans', 'Location', 'best');
grid on;

% Pompa Voltajı
subplot(2,1,2);
plot(t, u1_sim, 'm', 'LineWidth', 1.5);
title('Kontrol Girişi (Pompa Voltajı) - Chattering Analizi');
xlabel('Zaman (s)'); ylabel('Voltaj (V)');
ylim([-1 11]);
grid on;