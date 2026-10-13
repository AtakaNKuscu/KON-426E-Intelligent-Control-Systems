# Taşıma kaydı

**Tarih:** 13 Ekim 2026

Yerel ders klasöründen `intelligent-control-systems/` içine ayrı bir Git repo kopyası hazırlandı. Kaynakların asılları değiştirilmedi veya silinmedi. Ödev ZIP'lerindeki MATLAB dosyaları kökteki dosyalarla SHA-256 karşılaştırmasıyla aynı bulundu.

## Kapsam

| Kaynak | Repo konumu |
| --- | --- |
| `hw1_*.m`, `signal.mat`, Ödev 1 grafikleri ve raporu | `assignments/assignment-01/` |
| `hw2_*.m`, `USV_navigation.m`, FIS dosyaları, Ödev 2 grafikleri ve raporu | `assignments/assignment-02/` |
| `project_*.m`, `main.m`, proje MAT kayıtları ve grafikleri | `project/double-tank/` |
| `System1.p` | Yerel `assignments/assignment-02/dependencies/`; Git dışında |

Tek tek kaynak/hedef yolları, boyutları ve SHA-256 özetleri [file-manifest.json](file-manifest.json) içinde bulunur. Bu manifest ilk taşıma kaydıdır; gelecekte kod değiştirildiğinde başlangıç özeti olarak kalır. Git'in satır sonu normalizasyonu nedeniyle checkout sonrası metin dosyalarının byte özetleri farklılaşabilir.

## Yerelde bırakılanlar

- `Lecture*.pdf`: ders slaytları.
- `Assignment2.pdf`, `Intelligent_Control_HW1 (1).pdf`: öğretim elemanının ödev yönergeleri.
- Teslim ZIP'leri: içlerindeki rapor ve kodlar zaten ayrı dosyalarla taşındı.
- `NARMAL2_MRC _V2/`: başka bir modele ait Simulink referans paketi; kullanıcının kendi proje kaynaklarıyla karıştırılmadı.
- Simulink önbellekleri, autosave ve `.original` dosyaları.

Özgün ödev ve proje kodlarında algoritma veya parametre değişikliği yapılmadı. Repo düzeni için README dosyaları, `.gitignore`, `.gitattributes`, `setup_course.m` ve iş kaydı eklendi.
