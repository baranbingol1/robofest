# Algoritma Notlari

Bu dosya robotun parkur takip algoritmasini ve kararlarini kalici olarak not almak icin
tutulur. Parkur, kontrol veya gercek robot aktarimi degistikce bu dosya da
guncellenmelidir.

## 2026-06-13 Durumu

Webots parkuru su anda ortak siyah baslangic cizgisinden baslar ve sonra iki
hedef rotaya ayrilir:

- Kirmizi hedef: `center=(0.80, 0.36)`, `radius=0.18`
- Mavi hedef: `center=(-0.72, 0.52)`, `radius=0.18`

Kirmizi rota ile mavi rota goruntu alaninda birbirine yakin bolgelerden gecer.
Bu nedenle robot, ozellikle kirmizi ve mavi cizgilerin yakin oldugu yerde
kisa sure mavi cizgiye dogru donuyor gibi gorunebilir. Mevcut Webots
dogrulamasi yine de iki hedef icin gecmistir:

```text
red  -> reached_goal, final goal margin 0.021 m
blue -> reached_goal, final goal margin 0.020 m
```

Bu davranis kabul edilebilir bir simulasyon smoke sonucudur, fakat tek basina
"parkurdan ve renkten tamamen bagimsiz" bir algoritma kaniti degildir.

## Mevcut Kodun Mantigi

Ana controller:

- `ders_cizim/controllers/rgb_rl_controller/rgb_rl_controller.py`
- `ders_cizim/controllers/rgb_rl_controller/control_core.py`
- `ders_cizim/controllers/rgb_rl_controller/mission.py`

Is akisi:

1. Kamera goruntusunun alt bolgesi taranir.
2. Piksel seviyesi icin parlaklik/renk ayrimi yapilir.
3. Siyah ortak cizgi, hedef renk cizgisi ve diger guclu renk segmentleri
   satir satir gruplenir.
4. Secilen segmentin merkez x konumu kameranin merkezine gore hata olarak
   hesaplanir: `center_error`.
5. `heuristic_action()` bu hatayi esiklere boler ve araci sola/saga/duz surer.
6. Eylem diferansiyel suruse cevrilir: sol hiz ve sag hiz.
7. Hedef renk `red` veya `blue` olarak ayarlanir. Egitimde
   `MONSTERBORG_RL_START_COLOR=random` sadece bu iki renkten birini secer.

Mevcut heuristic bu parkurda calisir, ama parkur sekline ve hedef renk
mantigina hala baglidir. Ozellikle kesismeye yakin bolgelerde hedef renk
segmentasyonu guvenilir degilse kisa sureli yanlis dala bakabilir.

## Degerlendirme

Soru: "Su anki heuristic algoritma iyi mi?"

Kisa cevap:

- Webots smoke icin evet, calisiyor.
- Gercek robot ve yarismada savunulacak ana algoritma olarak tek basina yeterli
  degil.
- Daha saglam ve bilinir yaklasim: binary/renk maskeleme + cizgi merkezi
  bulma + PID/PD kontrol.

Neden:

- Heuristic esikler elle secili oldugu icin kamera acisi, isik, bant kalinligi
  ve kesismelerde hassas olabilir.
- Parkurdan bagimsizlik icin kontrol, "bu rota kirmizi S yapar" gibi bilgileri
  bilmemeli; yalnizca kameradaki takip edilecek cizginin merkeze gore hatasini
  kullanmalidir.
- Renkten bagimsizlik icin cizgi takip katmani rengi ogrenmemeli; once maske
  uretilmeli, sonra ayni merkez takip algoritmasi her maske icin calismalidir.

## Hedef Algoritma

Savunulabilir ana algoritma asagidaki gibi olmali:

1. Kamera goruntusunden bir ROI sec:
   - Genelde goruntunun alt yarisi veya alt ucte birlik kismi.
2. Takip edilecek cizgi icin binary maske uret:
   - Siyah cizgi icin koyuluk/esikleme.
   - Kirmizi veya mavi cizgi icin HSV/RGB renk maskesi.
   - Gercek robotta isik degisimi varsa adaptive threshold veya kalibre edilmis
     HSV araliklari kullan.
3. Maskedeki en anlamli blob/contour bolgesini sec:
   - En buyuk alan, kameraya en yakin satir, onceki merkeze yakinlik gibi
     genel kurallar kullan.
4. Cizgi merkezini hesapla:
   - Centroid: `cx = M10 / M00`
   - Hata: `error = cx - image_center_x`
5. Direksiyon komutunu PID veya en azindan PD ile hesapla:
   - `turn = Kp * error + Kd * (error - previous_error)`
   - `left = base_speed - turn`
   - `right = base_speed + turn`
6. Cizgi kaybolursa:
   - Kisa sure onceki hata yonunde ara.
   - Belirli sure bulunamazsa dur.
7. Hedef/secim mantigini cizgi takipten ayri tut:
   - `target_color=red` ise kirmizi maske aktif olur.
   - `target_color=blue` ise mavi maske aktif olur.
   - Cizgi takip kontrolu ayni kalir.

Bu yapi, parkur sekline degil kameradaki cizgi merkezine baglidir. Bu nedenle
S, viraj, farkli parkur ve benzer genislikteki bantlarda ayni temel kontrol
calisir.

## Renkten Bagimsizlik Notu

"Renkten bagimsiz" ifadesi iki farkli anlama gelebilir:

1. Kontrol algoritmasi renkten bagimsiz olsun:
   - Evet, hedef bu olmali. PID/centroid katmani sadece binary maske alir.
   - Maske siyah, kirmizi veya mavi olabilir; kontrol ayni kalir.
2. Robot hic hedef renk bilmeden dogru dali secsin:
   - Bu farkli bir problemdir. Dallanma varsa robotun gorev hedefi gerekir.
   - Hedef renk, tabela, komut, QR/AprilTag, siralama veya operator secimi gibi
     dis bir gorev bilgisi olmadan "dogru dal" tanimli degildir.

Bizim mevcut gorev tanimimiz: robot hedef olarak `red` veya `blue` alir ve o
hedefe gider. Bu dogru ve savunulabilir bir tanimdir.

## Egitim Notu

Kodda tabular Q-learning destegi vardir. Bu, heuristic'in uzerine deneysel bir
katmandir:

- Q-table yoksa veya durum degerleri esit ise sistem heuristic eylemi kullanir.
- Egitimde durumlar `center_error`, guven, cizgi genisligi, algilanan renk ve
  hedef renk gibi ayrik ozelliklere cevrilir.
- Bu yaklasim deney icin uygundur; fakat gercek robotta ilk guvenilir baseline
  olarak PID/PD cizgi takip tercih edilmelidir.

### Q-learning'in Dogru Yeri

Mevcut Q-learning konumu mantik olarak yanlis degil: kamera profilinden
ayrik durum uretiliyor ve sistem `hard_left`, `left`, `soft_left`, `straight`,
`soft_right`, `right`, `hard_right` aksiyonlarindan birini seciyor. Yani
Q-learning dogrudan direksiyon karar katmani olarak denenebilir.

Fakat bu projede daha savunulabilir ayrim su olmalidir:

- Alt seviye cizgi takip: centroid + PID/PD ile deterministik ve kararli
  tutulmali.
- Q-learning: kesismede dal secimi, cizgi kaybedince arama davranisi, hiz
  secimi veya PID/PD parametre ayari gibi daha ust kararlar icin kullanilmali.

Neden boyle:

- Cizgi takip surekli ve fiziksel bir kontrol problemidir; PID/PD bu is icin
  standart, aciklanabilir ve kalibre edilebilirdir.
- Tabular Q-learning, ayrik durum uzayinda calisir. Kamera acisi, isik,
  bant kalinligi ve motor farklari degisince ayni q-table kolayca genelleme
  kaybedebilir.
- Bu yuzden Q-learning'i "tek ana surucu beyni" yapmak yerine once PID/PD
  baseline'i saglamlastirip, RL'i karar/iyilestirme katmani yapmak daha dogru
  mimaridir.

### 2026-06-13 Kisa Egitim Denemesi

Bu makinede Webots ile kisa bir egitim smoke denemesi calistirildi. Repo
kirlenmesin diye q-table ve loglar `/tmp` altina yazildi:

```bash
MONSTERBORG_RL_MODE=train \
MONSTERBORG_RL_START_COLOR=random \
MONSTERBORG_RL_TRAIN_STEPS=600 \
MONSTERBORG_RL_EPISODE_STEPS=300 \
MONSTERBORG_RL_SAVE_INTERVAL=300 \
MONSTERBORG_RL_Q_TABLE=/tmp/robofest_train_probe_q_table.json \
MONSTERBORG_RL_STEP_LOG_PATH=/tmp/robofest_train_probe_steps.json \
MONSTERBORG_RL_SUMMARY_PATH=/tmp/robofest_train_probe_summary.json \
/Applications/Webots.app/Contents/MacOS/webots --mode=fast --stdout --stderr --minimize \
  ders_cizim/worlds/monsterborg_rgb_rl.wbt
```

Sonuc:

- Webots egitim modu calisti ve q-table kaydedildi.
- `training_steps=600`
- `episodes=2`
- `states=7`
- `nonzero_states=7`
- Bu kisa deneme hedefe ulasmadi: `terminal_reason=timeout`,
  `success=false`, `matched_target_ratio=0.0`.

Yorum: Su anda egitim teknik olarak baslatilabilir. Ancak 600 step sadece
"egitim mekanizmasi calisiyor mu" kontroludur; iyi bir q-table uretmez.
Gercek egitim icin daha uzun sure, varyant parkurlar, random start, motor
gurultusu/latency ve egitim sonrasi ayri test matrisi gerekir.

## Kabul Kriterleri

Bir degisikligi "saglam" saymak icin minimum kontroller:

- Unit testler gecmeli: `python3 -m unittest discover -s tests -v`
- Webots nominal smoke iki hedef icin gecmeli:
  - `--colors red blue`
  - `terminal=reached_goal`
  - `pass=1`
- Kamera goruntusu cizgi + zemin gormeli; sadece renk bandini doldurmamali.
- Fiziksel robotta once `hardware_probe`, sonra dusuk guclu motor kalibrasyonu,
  sonra sinirli hizli kapali cevrim calistirilmali.

## Kaynaklar

- OpenCV Image Thresholding: `cv.threshold`, adaptive threshold ve Otsu
  thresholding gibi binary maske uretme yontemlerini anlatir.
  https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
- OpenCV Contour Features: contour, moment ve centroid hesaplamasini
  aciklar. Centroid formulu: `Cx = M10 / M00`, `Cy = M01 / M00`.
  https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
- WPILib Introduction to PID: PID'in yaygin bir feedback controller oldugunu,
  error/setpoint kavramlarini ve P/I/D terimlerini aciklar.
  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html
- Sutton ve Barto, Reinforcement Learning: An Introduction: Q-learning ve
  tabular reinforcement learning icin temel kaynak.
  https://incompleteideas.net/book/the-book-2nd.html

## Sonraki Teknik Borc

- Mevcut `heuristic_action()` davranisini PID/PD tabanli merkeze takip
  kontrolu ile degistirmek veya en azindan opsiyonel hale getirmek.
- Kamera maskesini fiziksel robot icin kalibre edilebilir HSV/RGB araliklariyla
  ayirmak.
- Kesisimlerde hedef renk kilidini daha kararli yapmak icin "onceki hedef
  rengine histerezis" veya "en yakin/onceki merkezle tutarlilik" filtresi
  eklemek.
- ALGORITMA.md dosyasini her parkur/kontrol degisikliginde guncellemek.
