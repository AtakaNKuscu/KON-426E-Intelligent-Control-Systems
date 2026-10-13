# Dönem projesi — doğrusal olmayan çift tank kontrolü

Çift tank sisteminin açık çevrim davranışı, PI denetimi, fizik tabanlı ters dinamiklerle eğitilen sinir ağı ve NMPC öğretmeninden öğrenen ağ denetleyicileri. Dosya adlarındaki `PINN` etiketi orijinal deney adlandırmasıdır; sürümlerin eğitim yöntemleri farklıdır.

## Başlangıç

```matlab
setup_course('project');
project_system;  % Açık çevrim model
project_PI;      % PI referans denetleyici
```

Bu iki script temel MATLAB ile çalışır. Sinir ağı deneyleri Deep Learning Toolbox; NMPC öğretmeni Optimization Toolbox gerektirir.

## Deney aileleri ve çalıştırma sırası

| Dosya | Açıklama / önkoşul |
| --- | --- |
| `src/project_system.m` | Doğrusal olmayan sistemin açık çevrim simülasyonu |
| `src/project_PI.m` | PI referans denetleyici |
| `src/project_PINN_v2.m` | Fizik tabanlı ters dinamiklerle 3 girişli `net` eğitimi ve kapalı çevrim deneyi; ağı çalışma alanında bırakır |
| `src/project_PINN_v2_test.m` | Nominal basamak testleri; önce `project_PINN_v2` çalıştırılmalı |
| `src/project_stability.m` | Aynı 3 girişli ağın yerel kararlılık incelemesi; çalışma alanında `net` gerekir |
| `src/project_test.m` | Değişen referanslı basamak testi; çalışma alanında 3 girişli `net` veya çalışma klasöründe `trained_NN_controller.mat` gerekir |
| `src/project_test2.m` | Gürültülü ölçüm / filtreleme karşılaştırması; aynı 3 girişli ağı gerektirir |
| `src/main.m` | Basamak, merdiven, bozucu ve gürültü senaryoları; aynı 3 girişli ağı gerektirir |
| `src/project_PINN_v2_test2.m` | Çalışma klasöründeki `trained_pinn_controller.mat` ile deney; aşağıdaki nota bakın |
| `src/project_PINN.m` | 7 girişli tam durum NN ile yaklaşık NMPC; varsayılan olarak öğretmen veri setini yeniden üretir ve ağı yeniden eğitir |
| `src/project_NN_MPC_test.m` | Saklanan 6 girişli çıkış geri beslemeli NN-MPC modelinin testleri; eğitim yapmaz |

Üç girişli ağ ile örnek sıra:

```matlab
project_PINN_v2;
project_PINN_v2_test;
project_stability;
project_test;
project_test2;
main;
```

`project_PINN_v2` ağı otomatik kaydetmez. Sonraki oturumlarda kullanmak için bölüm çalışma klasöründe `save('trained_NN_controller.mat', 'net')` çalıştırabilirsiniz. Bu dosya üretilen çıktı olarak Git dışında kalır.

Saklanan çıkış geri beslemeli modeli test etmek için:

```matlab
setup_course('project');
project_NN_MPC_test;
```

`setup_course` veri klasörünü MATLAB yoluna eklediği için `load` model dosyasını bulur. `project_PINN` içindeki `generateNewDataset = true` ayarı uzun süren NMPC optimizasyonları başlatır. Saklanan veri setiyle çalışmak isterseniz ayarı `false` yapın; ağ yine yeniden eğitilir.

## Veri ve sonuç arşivi

`data/` altında mevcut eğitim verileri, model dosyaları ve simülasyon sonuçları; `figures/` altında JPG grafikler ile basamak / NN-PI karşılaştırma PNG'leri bulunur. Bunlar farklı geliştirme aşamalarının kayıtlarıdır. Her modelin eğitim scripti mevcut değildir; ağları yalnızca dosya adına bakarak birbirinin yerine kullanmayın.

- `Robust_FullState_NN_MPC_Dataset.mat` ve `Robust_FullState_NN_based_MPC_Final.mat`: `project_PINN` ailesi.
- `Output_Feedback_NN_based_MPC_DoubleTank_Final.mat`: `project_NN_MPC_test` tarafından yüklenen model. Üretici eğitim scripti kaynak klasörde yoktur.
- `step_test_results.mat` ve `noise_rejection_results.mat`: mevcut test sonuçları.
- `trained_pinn_controller.mat`: önceki eğitim kaydı. `project_PINN_v2_test2` dosya varlığını çalışma klasöründe kontrol eder; bu kaydı kullanmak için `copyfile('data/trained_pinn_controller.mat', 'trained_pinn_controller.mat')` gerekir. Çalışabilirliği doğrulama kaydında belirtilir.
- Diğer `.mat` dosyaları geçmiş denemeler olarak korunmuştur; isimleri özgün haliyle bırakılmıştır.

Test scriptleri yeni `.mat` dosyalarını ve `figures_step_test/`, `figures_noise_rejection/` gibi klasörleri bölümün çalışma klasörüne yazar. Bunlar mevcut arşiv grafiklerini değiştirmez ve Git dışında tutulur.

Yerel klasörde bulunan başka bir modele ait `NARMAL2_MRC _V2` referans paketi bu repoya dahil edilmemiştir. Proje raporu yerel kaynaklarda bulunmamaktadır. Eksikler [iş listesinde](../../docs/ROADMAP.md) izlenir.
