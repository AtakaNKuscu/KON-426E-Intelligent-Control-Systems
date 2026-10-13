# Ödev 1

Doğrusal olmayan regresyon, model öngörülü kontrol ve adaptif PID çalışmaları. [Teslim raporu](reports/Intelligent_Control_Systems_HW1.pdf) ve [mevcut grafikler](figures/) birlikte arşivlenmiştir.

| Dosya | İçerik | Bağımlılık |
| --- | --- | --- |
| `src/hw1_part1.m` | Sinüs modelinin GD, momentum ve Newton ile kestirimi; ilk %80 eğitim / son %20 test | MATLAB, `data/signal.mat` |
| `src/hw1_part2.m` | Ayrık zamanlı doğrusal sistemde kayan ufuklu kontrol | Optimization Toolbox (`fminunc`) |
| `src/hw1_part3.m` | Sabit PID ve adaptif PID benzeri denetleyicinin karşılaştırılması | MATLAB |

Repo kökünde:

```matlab
setup_course('hw1');
hw1_part1;  % Regresyon
hw1_part2;  % MPC
hw1_part3;  % PID karşılaştırması
```

Scriptler bağımsızdır; her biri çalışma alanını ve açık figürleri temizler. Grafikler ekranda oluşturulur; `figures/` altındaki JPG dosyaları önceki teslimin saklanan çıktılarıdır ve scriptler tarafından otomatik güncellenmez.

`signal.mat`, dersin regresyon deneyi için sağlanan veri dosyasıdır. Kaynak kodlar orijinal teslim sürümüyle aynıdır; yalnızca klasör konumları düzenlenmiştir.
