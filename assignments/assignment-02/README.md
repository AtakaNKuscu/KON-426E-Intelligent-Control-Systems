# Ödev 2

NARX ile tanılama, küresel tankta bulanık kazanç ayarlamalı PI ve insansız su üstü aracı (USV) navigasyonu. [Teslim raporu](reports/Intelligent_Control_Systems_HW2.pdf) ve [grafikler](figures/) burada saklanır.

| Dosya | Görev | Gereksinim |
| --- | --- | --- |
| `src/hw2_part1.m` | NARX gecikme seçimi, eğitim ve doğrulama | Deep Learning Toolbox, harici `System1.p` |
| `src/hw2_fixedPI.m` | Küresel tankın sabit PI ile kontrolü | MATLAB |
| `src/hw2_FGS_PI_FIS.m` | Bulanık PI kazanç ayarlayıcısını oluşturma | Fuzzy Logic Toolbox |
| `src/hw2_PI_FIS_action.m` | Bulanık PI kapalı çevrim simülasyonu | Fuzzy Logic Toolbox, `FGS_PI_Controller.fis` |
| `src/hw2_USV_FIS_create.m` | USV hız denetleyicisini oluşturma | Fuzzy Logic Toolbox |
| `src/USV_navigation.m` | Yol noktaları arasında USV navigasyonu | Fuzzy Logic Toolbox, `USV_Speed_Controller.fis` |

Repo kökünde:

```matlab
setup_course('hw2');
hw2_fixedPI;
hw2_FGS_PI_FIS;
hw2_PI_FIS_action;
hw2_USV_FIS_create;
USV_navigation;
```

`controllers/` içinde teslimden kalan iki FIS dosyası bulunur. Oluşturucu scriptler çalıştırıldığında güncel çalışma klasörüne yeni `.fis` dosyaları yazar; MATLAB önce bu yeni dosyaları kullanır. Yeni tanımı repoya almak için gözden geçirip `controllers/` altına taşıyın.

NARX deneyi için ders kapsamında edindiğiniz `System1.p` dosyasını `dependencies/` içine yerleştirin, ardından `hw2_part1` çalıştırın. Scriptteki `studentID` sistemin kişiye özgü davranışını belirler; farklı kimlik farklı sistem ve sonuç üretir. NARX eğitimi diğer örneklerden daha uzun sürebilir.

Mevcut grafikler teslim çıktılarıdır; tümü scriptler tarafından diske otomatik kaydedilmez. Kaynak kodlar teslim sürümünden değiştirilmeden taşınmıştır.
