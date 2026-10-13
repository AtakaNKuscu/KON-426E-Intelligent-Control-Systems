# Doğrulama kaydı

**Tarih:** 13 Ekim 2026  
**Ortam:** MATLAB 25.2.0.3055257 (R2025b) Update 2, Windows  
**Toolbox sürümleri:** Optimization Toolbox 25.2, Deep Learning Toolbox 25.2, Fuzzy Logic Toolbox 25.2.

## Taşıma ve dosya bütünlüğü

- İlk kopyalama sırasında **98 dosya**, toplam **11.791.501 byte**, kaynak/hedef SHA-256 karşılaştırmasıyla doğrulandı. Bunlardan `System1.p` yalnızca yerel bağımlılıktır; kalan 97 kaynak dosya Git kapsamındadır.
- Ödev ZIP'lerindeki tüm MATLAB kaynakları kökteki sürümleriyle aynı bulundu.
- MATLAB, proje arşivindeki **13 MAT dosyasının tamamının** değişken listesini okuyabildi. Bu kontrol bütün eski modellerin kapalı çevrim performansını doğrulamaz.

## MATLAB çalıştırma kontrolleri

`setup_course` ile ilgili bölüme geçildikten sonra, görünmeyen figürlerle aşağıdaki **11 script hatasız tamamlandı**:

| Bölüm | Çalıştırılan scriptler |
| --- | --- |
| Ödev 1 | `hw1_part1`, `hw1_part2`, `hw1_part3` |
| Ödev 2 | `hw2_fixedPI`, `hw2_FGS_PI_FIS`, `hw2_PI_FIS_action`, `hw2_USV_FIS_create`, `USV_navigation` |
| Proje | `project_system`, `project_PI`, `project_NN_MPC_test` |

MPC durum/kontrol dizileri, PID yanıtları ve bulanık PI'nin son seviyesi/kontrol dizisi için sonlu değer kontrolleri de geçti. Üretici FIS scriptleri ile bunları kullanan simülasyonlar aynı çalışma klasöründe sırayla çalıştırıldı. Saklanan çıkış geri beslemeli NN-MPC modelinin yüklenmesi ve farklı referans, başlangıç koşulu, gürültü ve bozucu senaryolarında çıkarım yapması doğrulandı.

## Gözlemler ve kapsam sınırları

- Ödev 1 regresyonunda GD ve momentumun test MSE'si yaklaşık `0.016294`; mevcut Newton başlangıcıyla test MSE'si yaklaşık `1.530192` çıktı. Hatasız çalıştırma, yöntemlerin aynı çözüme yakınsadığını göstermez; orijinal algoritma/parametreler değiştirilmedi.
- USV örneği 5/5 yol noktasına ulaştı; son uzaklık `0.1478 m`, kabul yarıçapı `0.15 m`.
- Projenin mevcut PI denetleyicisinde `0.25 m` referansı için son mutlak hata yaklaşık `0.011723 m`.
- Saklanan NN-MPC modelinin bazı bozucu senaryolarında %2 yerleşme süresi `NaN`: simülasyon ufku içinde yerleşme koşulu sağlanmıyor. Bu metrik, scriptin çalıştırma hatası değildir.
- NARX eğitimi (`hw2_part1`), üç girişli ağın yeniden eğitimi (`project_PINN_v2`) ve NMPC öğretmeniyle yeniden eğitim (`project_PINN`) bu taşıma kontrolünde çalıştırılmadı.
- Üç girişli ağı gerektiren proje testleri bu ilk kontrolün kapsamı dışındadır. `trained_pinn_controller.mat` içinde `net` değişkeni bulunduğu doğrulandı; modelin bu testlerle davranış uyumu ayrıca incelenmelidir.
- Yeni kontrol çıktıları Git dışında bırakıldı; teslim raporları ve arşivlenmiş grafikler yeniden üretilmiş gibi sunulmadı.

Bu kayıt, klasör düzeni ve çalıştırma bağımlılıklarının kontrolüdür. Bilimsel sonuçların tamamının bağımsız yeniden üretimi değildir.
