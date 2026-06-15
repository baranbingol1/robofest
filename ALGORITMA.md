# Algoritma Notları

Bu dosya `ders_cizim` simülasyonundaki MonsterBorg çizgi takip algoritmasını,
sunumda savunulabilecek akademik çerçeveyi ve gerçek robota aktarım sınırlarını
özetler. Dosya UTF-8 Türkçe tutulur.

## Güncel Mimari

Robot artık üç katmanlı bir yaklaşımla çalışır:

1. **Görüntü işleme katmanı**
   - Kamera görüntüsünden çizgi/renk segmentleri çıkarılır.
   - Seçilen segmentin kamera merkezine göre hatası `center_error` olarak
     hesaplanır.
   - Güven skoru, çizgi genişliği, hedef renk eşleşmesi ve görünürlük bilgisi
     tek bir profil yapısında toplanır.

2. **Alt seviye çizgi takip katmanı**
   - Motor komutu artık varsayılan çizgi takip davranışında eşik tabanlı
     keskin dönüşlerden değil, PD geri beslemesinden gelir.
   - Denklem:

     ```text
     turn = Kp * center_error + Kd * (center_error - previous_error)
     left_speed  = base_speed - turn
     right_speed = base_speed + turn
     ```

   - Varsayılan kazançlar:
     - `Kp = 1.8`
     - `Kd = 0.45`
   - Kazançlar gerçek robotta ayarlanabilsin diye ortam değişkenleriyle
     değiştirilebilir:
     - `MONSTERBORG_RL_LINE_KP`
     - `MONSTERBORG_RL_LINE_KD`

3. **Üst seviye davranış seçimi**
   - Tabular Q-learning, ham teker hızını seçmez.
   - Q-learning yalnızca davranış seçer:
     - `follow_line`: PD ile çizgiyi takip et.
     - `search_target`: hedef dala kontrollü arama/bias uygula.
     - `slow_follow`: aynı PD kontrolünü daha düşük hızla çalıştır.
   - Q-table yoksa, durum bilinmiyorsa veya güvenlik kısıtları devredeyse
     deterministik fallback kullanılır.

Bu ayrım önemlidir: sürekli ve fiziksel olan çizgi takip işi PD kontrolcüye
bırakılır; Q-learning ise ayrık ve açıklanabilir görev davranışları arasında
seçim yapar.

## Neden Bu Algoritma?

Bu proje bir akademik çalışma değil, ders projesidir. Bu yüzden iddiamız şu
olmalı:

> Webots üzerinde çalışan oyuncak bir robot simülasyonu kurduk. Sistem,
> literatürdeki kamera tabanlı çizgi takip ve PID/PD kontrol yaklaşımından
> esinlenen bir alt seviye kontrolcü ile, tabular Q-learning kullanan hafif bir
> davranış seçici katmanı birleştirir.

Bu iddia kodla uyumludur ve abartılı değildir.

Derin öğrenme veya uçtan uca deep RL bu ortam için gereksizdir. Kamera açısı,
ışık, bant kalınlığı, zemin parlaklığı ve motor farkları değiştiğinde deep RL
simülasyondan gerçek robota zor taşınır. PD kontrol ise hem ayarlanabilir hem de
sunumda kolay açıklanabilir.

## Mevcut Kod Haritası

Ana dosyalar:

- `ders_cizim/controllers/rgb_rl_controller/rgb_rl_controller.py`
- `ders_cizim/controllers/rgb_rl_controller/control_core.py`
- `ders_cizim/controllers/rgb_rl_controller/hardware_runner.py`
- `ders_cizim/controllers/rgb_rl_controller/mission.py`

Önemli noktalar:

- `analyze_rgb_camera(...)` kamera görüntüsünden çizgi profilini üretir.
- `line_follow_command(...)` PD tabanlı sol/sağ motor komutunu üretir.
- `option_to_command(...)` Webots option katmanını gerçek motor komutuna
  çevirir.
- `QPolicy` tabular Q-learning güncellemesini yapar.
- `hardware_runner.py` gerçek Raspberry Pi koşusunda aynı `line_follow_command`
  fonksiyonunu kullanır; böylece simülasyon ve fiziksel robot mantığı ayrılmaz.

Eski `heuristic_action(...)` tamamen silinmedi. Hâlâ legacy/direct policy,
arama davranışı ve bazı fallback durumlarında kullanılır. Ancak varsayılan
option tabanlı çizgi takipte ana motor komutu PD kontrolcüden gelir.

## Option Tanımı

Bu projede "option" kelimesini Sutton, Precup ve Singh'in kuramsal çerçevesine
yakın ama sadeleştirilmiş anlamda kullanıyoruz. Tam akademik SMDP/option
kanıtları yaptığımızı iddia etmiyoruz.

Kullandığımız pratik option'lar:

| Option | Başlatma koşulu | Politika | Bitiş koşulu |
| --- | --- | --- | --- |
| `follow_line` | Çizgi görünür ve güven yeterli | PD çizgi takip | Hedef arama, düşük güven veya terminal |
| `search_target` | Hedef dal aranmalı | Hedef renge göre kontrollü bias | Hedef çizgi kilidi veya güvenlik terminali |
| `slow_follow` | Hata büyük, çizgi zayıf veya güven düşük | PD çizgi takip, düşük hız | Güvenli takip geri gelince |

Bu yapı, Q-learning'i ham direksiyon yerine zamana yayılan davranışlar arasında
seçim yapan daha güvenli bir katmana taşır.

## Renk ve Hedef Mantığı

"Renkten bağımsız" ifadesini doğru tanımlamak gerekir:

- Alt seviye kontrol renkten bağımsızdır. PD kontrolcü yalnızca `center_error`
  görür; bu hata siyah, kırmızı veya mavi maskeden gelebilir.
- Görev seçimi renkten bağımsız değildir. Parkur dallanıyorsa robotun hangi
  hedefe gideceğini bilmesi gerekir. Bu hedef `red`, `blue` veya özel siyah
  test koşusu olabilir.

Yani doğru iddia şudur: çizgi takip kontrolcüsü maske renginden bağımsızdır;
görev hedefi ise bilinçli olarak renk parametresiyle verilir.

## Gerçek Robot Aktarımı

Bu değişiklik fiziksel robotu doğrulamaz. Gerçek robot için yapılması gerekenler:

1. Kamera montajını sabitle.
2. `hardware_probe` ile çizgi genişliği, görünürlük oranı ve güven değerlerini
   ölç.
3. Motor yönlerini ve hız ölçeklerini düşük güçte kalibre et.
4. `MONSTERBORG_HARDWARE_OUTPUT_LIMIT` değerini düşük başlat.
5. `MONSTERBORG_RL_LINE_KP` ve `MONSTERBORG_RL_LINE_KD` değerlerini gerçek
   zemin, ışık ve bant kalınlığına göre ayarla.
6. Önce düz çizgi, sonra yumuşak viraj, sonra dallanma testine geç.

Gerçek robotta ilk deneme için önerilen başlangıç:

```powershell
$env:MONSTERBORG_HARDWARE_OUTPUT_LIMIT='0.35'
$env:MONSTERBORG_RL_LINE_KP='1.2'
$env:MONSTERBORG_RL_LINE_KD='0.25'
```

Simülasyondaki varsayılan kazançlar gerçek robota birebir taşınmak zorunda
değildir. Gerçek robotta daha düşük kazançla başlamak daha güvenlidir.

## Doğrulama Kriterleri

Bir değişiklik tamam sayılmadan önce en az şu kontroller yapılmalıdır:

- Unit testler:

  ```powershell
  python -m unittest discover -s tests -v
  ```

- Normal Webots smoke matrisi:
  - kırmızı hedef
  - mavi hedef
  - siyah sembolik hedef

- Kamera pozu, motor asimetrisi, başlangıç rastgeleliği ve gürültü matrisleri.

- Stress texture matrisi.

Başarılı koşu için:

- süreç dönüş kodu `0`,
- `passed_smoke_gate=true`,
- terminal neden `reached_goal`,
- `off_board=false`,
- `timed_out=false`,
- `lost_line=false`.

Bu kriterler simülasyon içindir. Fiziksel robot için ayrıca düşük hızda güvenli
kapalı çevrim testleri gerekir.

## Sunumda Kullanılacak Dürüst Cümle

Şu ifade doğru ve savunulabilir:

> Robotumuz kamera görüntüsünden çizgi merkez hatasını çıkarır. Alt seviyede
> PD tabanlı diferansiyel sürüş kontrolü kullanır. Üst seviyede ise küçük bir
> tabular Q-learning katmanı, `follow_line`, `search_target` ve `slow_follow`
> gibi davranışlar arasında seçim yapar. Bu yaklaşım kamera tabanlı PID/PD çizgi
> takip çalışmalarından ve tabular Q-learning/option literatüründen esinlenir;
> yeni bir akademik yöntem iddiası taşımaz.

## Kaynak Çerçevesi

- Farkh ve Aljaloud: kamera tabanlı çizgi takip ile PID kontrolün birlikte
  kullanılmasına iyi bir referanstır.
- Li: Raspberry Pi, Pi camera, OpenCV renk/threshold işleme ve PID hareket
  kontrolü için uygun bir yakın çalışmadır.
- Saadatmand ve arkadaşları: çizgi takip robotunda Q-learning kullanımını
  göstermek için uygundur.
- Watkins ve Dayan: tabular Q-learning'in klasik kaynağıdır.
- Sutton, Precup ve Singh: davranışları option olarak çerçevelemek için
  kullanılabilir.

Bu kaynakları "aynısını yaptık" diye değil, "yaklaşımımız bu fikirlerden
esinlenir" diye kullanmalıyız.
