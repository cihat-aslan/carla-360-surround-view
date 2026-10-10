# 360° Automotive Surround View (AVM) with Analytical IPM & ROS 2 Bridge

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy%20%2F%20Humble-blue.svg)](https://docs.ros.org/)
[![Platform](https://img.shields.io/badge/Simulator-CARLA%200.9.16-green.svg)](https://carla.org/)
[![OpenCV](https://img.shields.io/badge/CV-remap%20LUT-orange.svg)]()

CARLA simülatöründe 4 adet geniş açılı (120° FOV) RGB kamera kullanılarak, kamera içsel/dışsal parametrelerinden analitik türetilmiş zemin homografisi ile çalışan gerçek zamanlı Çevre Görüş Sistemi (Around View Monitoring - AVM) ve ROS 2 algı köprüsü.

---

## 📸 Görselleştirme & Demo

| 360° Kuşbakışı Kalibre Zemin Görünümü | ROS 2 TF Ağacı Hiyerarşisi |
| :---: | :---: |
| ![360 Surround Demo](assets/demo.gif) | ![TF Tree](assets/tf_tree.png) |

---

## ⚙️ Temel Mühendislik Yaklaşımı

* **Analitik Zemin Homografisi (Zero Magic Numbers):** Piksel koordinatları sezgisel seçilmez. CARLA `Transform` (extrinsic) ve kameranın `FOV` değerlerinden türetilen içsel matris ($K$) kullanılarak $Z=0$ zemin düzlemi için analitik homografi çıkarılır.
* **Önceden Hesaplanmış LUT (`cv2.remap`):** Kamera yerleşimi sabit olduğundan dönüşüm haritaları döngü öncesinde bir kez üretilir. Çalışma anında sadece arama tablosu üzerinden haritalama yapılarak işlem yükü minimize edilir.
* **Açısal Alfa Harmanlama (Angular Alpha Blending):** Sektör birleşimlerinde dikiş hatlarını gizlemek yerine $12^\circ$ açısal geçişle komşu kameralar pürüzsüzce harmanlanır.
* **ROS 2 Düğümü & REP-103 Uyumlu TF:** Unreal sol-el koordinatları kuaterniyon matematiği ile ROS standart optik çerçevesine (`cam_<ad>_optical`) dönüştürülür ve `/tf_static` ile yayınlanır.
* **Sayısal Doğrulama Metriği (Overlap Error):** Komşu kameraların örtüşen bölgelerindeki $L_1$ yoğunluk farkı anlık olarak hesaplanır (ortalama hata: **< 12 / 255**).

---

## 📊 Performans Değerleri

| Metrik | Değer | Açıklama |
|---|---|---|
| **Pipeline Latency** | ~28 - 32 ms | 4x remap + harmanlama (~33+ FPS) |
| **Sim Loop** | ~5.4 FPS | CARLA off-screen render ve veri aktarımı |
| **Kapsama Alanı** | 20 m x 20 m (±10 m) | 40 px/m çözünürlük |
| **Tuval Boyutu** | 800 x 800 | 3 kanallı BGR / `sensor_msgs/Image` |
| **Örtüşme Hatası** | 7.4 - 12.7 / 255 | Geometrik hizalama doğrulaması |

---

## 🚀 Kurulum ve Çalıştırma

### 1. CARLA Simülatörünü Başlatın
```bash
./CarlaUE4.sh -RenderOffScreen -nosound
