% --- INTELLIGENT CONTROL: DATA-DRIVEN PHYSICS-INFORMED NEURAL CONTROLLER ---
% Yöntem: State-Space Generalization & Inverse Dynamics
clear; clc; close all;

%% 1. Sistem Parametreleri (PDF Plant Model)
c1 = 1.035454e-2;
c2 = 1.143e-3;
H_max = 0.6; % Tankların maksimum boyu [cite: 56]
V_max = 10;  % Pompa maksimum voltajı [cite: 57]

%% 2. AŞAMA 1: FİZİK BİLGİLİ VERİ SETİ ÜRETİMİ (Generalization)
disp('Sistemin Durum-Uzayı (State-Space) Haritası Çıkarılıyor...');
N_samples = 50000; % Ağın generalize etmesi için 50 bin farklı senaryo

% Rastgele Başlangıç Koşulları ve Hedefler
X_data = zeros(3, N_samples); % Girdiler: [h1; h2; h_ref]
Y_data = zeros(1, N_samples); % Çıktı: [u1]

for i = 1:N_samples
    % 0 ile 0.6m arası rastgele durumlar
    h1 = rand() * H_max;
    h2 = rand() * H_max;
    href = rand() * H_max;
    X_data(:, i) = [h1; h2; href];
    
    % --- TERSİNE FİZİK (INVERSE DYNAMICS) UZMAN SİSTEMİ ---
    % Sistemin PDF Eq 5.10 denklemi üzerinden ideal akış hesabı [cite: 70]
    error_h2 = href - h2;
    desired_dh2 = 0.05 * error_h2; % İkinci tank için yumuşak hedef hızı
    
    % Gerekli akış miktarı (Flow = c1*sign*sqrt)
    desired_flow = desired_dh2 + c1 * sqrt(h2); 
    
    % Gerekli akışı sağlamak için Tank 1'in olması gereken seviyesi (h1_star)
    sign_flow = sign(desired_flow);
    h1_star = h2 + sign_flow * (desired_flow / c1)^2;
    h1_star = max(0, min(h1_star, H_max)); % Fiziksel limitleri aşamaz
    
    % Tank 1'i h1_star hedefine götürecek hız (PDF Eq 5.9) [cite: 68, 69]
    error_h1 = h1_star - h1;
    desired_dh1 = 0.5 * error_h1; % Birinci tank için agresif hedef hızı
    
    % Gerekli Pompa Voltajı (u1)
    u1_req = (desired_dh1 + desired_flow) / c2;
    
    % Pompa 0-10V arası çalışabilir [cite: 57]
    Y_data(i) = max(0, min(u1_req, V_max)); 
end

%% 3. AŞAMA 2: SİNİR AĞININ EĞİTİLMESİ (Plant Inversion Controller)
disp('Neural Network Eğitiliyor (Levenberg-Marquardt)...');
% 2 gizli katmanlı, 32 ve 16 nöronlu güçlü bir ağ
net = fitnet([32, 16]); 
net.trainParam.showWindow = false; % Eğitim penceresini gizle
net.trainParam.epochs = 500;

% Ağı verilerle eğit (Saniyeler sürecek ve mükemmel öğrenecek)
[net, tr] = train(net, X_data, Y_data);
disp(['Eğitim Bitti. Son Hata (MSE): ', num2str(tr.best_perf)]);

%% 4. AŞAMA 3: KAPALI ÇEVRİM TESTİ (Disturbance & Noise Bonus)
disp('Gürültülü ve Bozucu Etkili Kapalı Çevrim Simülasyonu Başlıyor...');
dt = 0.1;
t_end = 400;
t = 0:dt:t_end;
N = length(t);

h1_sim = zeros(1, N);
h2_sim = zeros(1, N);
u1_sim = zeros(1, N);
h_ref = 0.3; % Hedef 0.3 metre

for k = 1:(N-1)
    % 1. SENSÖR OKUMASI VE NOISE REJECTION BÖLÜMÜ
    % Sensörlerde +/- 2mm beyaz gürültü (white noise) var
    h1_read = h1_sim(k) + 0.002 * randn(); 
    h2_read = h2_sim(k) + 0.002 * randn();
    
    % 2. NEURAL CONTROLLER DEVREDE (Kapalı Çevrim)
    % Ağ, genelleştirdiği state-space tecrübesine dayanarak voltajı anında üretir
    u1_sim(k) = net([h1_read; h2_read; h_ref]);
    
    % Fiziksel pompa sınırı [cite: 57]
    u1_sim(k) = max(0, min(u1_sim(k), V_max)); 
    
    % 3. GERÇEK FİZİKSEL SİSTEM (PLANT)
    diff_h = h1_sim(k) - h2_sim(k);
    sign_val = sign(diff_h);
    sqrt_diff = sqrt(abs(diff_h));
    
    dh1 = -c1 * sign_val * sqrt_diff + c2 * u1_sim(k);
    dh2 =  c1 * sign_val * sqrt_diff - c1 * sqrt(h2_sim(k));
    
    h1_sim(k+1) = max(0, min(h1_sim(k) + dt * dh1, H_max));
    h2_sim(k+1) = max(0, min(h2_sim(k) + dt * dh2, H_max));
    
    % 4. DISTURBANCE HANDLING BÖLÜMÜ (Bozucu Etki)
    % t = 200. saniyede ikinci tankta büyük bir sızıntı oluyor (0.15m su kaybı)
    if abs(t(k) - 200) < dt
        h2_sim(k+1) = max(0, h2_sim(k+1) - 0.15); 
    end
end
u1_sim(end) = u1_sim(end-1);

%% 5. GRAFİKLERİ ÇİZ
figure('Name', 'Neural Controller Performance', 'Position', [100, 100, 850, 650]);

subplot(2,1,1);
plot(t, h2_sim, 'b', 'LineWidth', 2); hold on;
plot(t, h1_sim, 'r--', 'LineWidth', 1.5);
plot(t, h_ref * ones(1, N), 'k:', 'LineWidth', 2);
xline(200, 'r-', 'BÜYÜK SIZINTI (Disturbance)', 'LabelVerticalAlignment', 'bottom', 'LineWidth', 2);
title('Kapalı Çevrim (Closed-Loop) Neural Controller Yanıtı');
xlabel('Zaman (s)'); ylabel('Su Seviyesi (m)');
legend('Tank 2 (h_2)', 'Tank 1 (h_1)', 'Referans (Step Input)', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(t, u1_sim, 'm', 'LineWidth', 1.5);
title('Kontrol Girişi (Pompa Voltajı)');
xlabel('Zaman (s)'); ylabel('Voltaj (V)');
ylim([-1 11]); grid on;