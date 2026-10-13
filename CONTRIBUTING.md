# Çalışma düzeni

## Dosyaları yerleştirme

Her ödev kendi numaralı klasöründe, dönem projesi `project/double-tank/` altında tutulur:

- `src/`: MATLAB kaynak kodu.
- `data/`: deneyin girdileri ve tekrar kullanılacak model / sonuç kayıtları.
- `controllers/`: Ödev 2'nin FIS tanımları.
- `figures/`: rapora veya karşılaştırmaya alınacak seçilmiş grafikler.
- `reports/`: teslim veya güncel rapor PDF'leri.

Yeni bir ödev gelirse `assignments/assignment-03/` gibi sıradaki numarayı kullanın ve ana README tablosunu güncelleyin. Geçici dosyaları, ders slaytlarını ve aynı dosyaların ZIP kopyalarını eklemeyin.

## Bir değişikliği tamamlama

1. [İş listesinden](docs/ROADMAP.md) bir iş seçin veya açıklamalı bir GitHub issue açın.
2. Kapsamı büyüyen işler için `hw2/...`, `project/...` veya `docs/...` adlı kısa bir branch açın.
3. İlgili bölümde `setup_course` ile ortamı kurun ve etkilenen deneyi çalıştırın.
4. Referans sinyali, başlangıç koşulu, örnekleme süresi, rastgelelik tohumu ve modelin giriş yapısını kaydedin. Eğitim tekrar yapılmadıysa bunu açıkça yazın.
5. Gerekli grafik/verileri ilgili alt klasöre alın ve bölüm README'sini güncelleyin.
6. `git status` ve `git diff --stat` ile kapsamı gözden geçirin; ilişkili dosyaları tek bir açıklayıcı commit içinde kaydedin.

Örnek commit mesajları:

```text
docs: document double-tank experiment dependencies
hw2: compare fuzzy PI gains at different operating points
project: add disturbance rejection results for inverse controller
```

İlk taşıma kaynak kodların davranışını değiştirmez. Sonraki iyileştirmelerde kod değişikliği ile deney sonucu arasındaki ilişki commit açıklamasında belirtilmelidir. `main` branch'i incelenebilir ve tekrar çalıştırma talimatları güncel durumda tutulur.
