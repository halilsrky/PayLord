# 🚀 PayLord - Payload Kartı Yazılımı

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![STM32](https://img.shields.io/badge/STM32-F446RE-green.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)
[![TEKNOFEST](https://img.shields.io/badge/TEKNOFEST-2024--2025-red.svg)](https://www.teknofest.org/)

**Aeronist Aerospace** takımının TEKNOFEST 2024-2025 Orta İrtifa Roket Yarışması için geliştirdiği Payload (Faydalı Yük) kartı gömülü yazılımı.

---

## 📋 İçindekiler

- [Proje Hakkında](#-proje-hakkında)
- [Donanım Özellikleri](#-donanım-özellikleri)
- [Yazılım Mimarisi](#-yazılım-mimarisi)
- [Sensörler ve Modüller](#-sensörler-ve-modüller)
- [Özellikler](#-özellikler)
- [Kurulum](#-kurulum)
- [Derleme ve Yükleme](#-derleme-ve-yükleme)
- [Kullanım](#-kullanım)
- [Kod Yapısı](#-kod-yapısı)
- [Katkıda Bulunma](#-katkıda-bulunma)
- [Lisans](#-lisans)
- [İletişim](#-iletişim)

---

## 🎯 Proje Hakkında

PayLord, roket uçuşu sırasında kritik verilerin toplanması, işlenmesi ve iletilmesi için tasarlanmış gelişmiş bir aviyonik sistemidir. STM32F446RE mikrodenetleyici tabanlı bu sistem, gerçek zamanlı veri toplama, sensör füzyonu, uçuş aşaması tespiti ve kablosuz telemetri özellikleri sunar.

### Yarışma Bilgileri
- **Yarışma:** TEKNOFEST 2024-2025 Orta İrtifa Roket Yarışması
- **Takım:** Aeronist Aerospace
- **Kategori:** Orta İrtifa Roket

---

## 🔧 Donanım Özellikleri

### Ana İşlemci
- **Mikrodenetleyici:** STM32F446RET6
  - ARM Cortex-M4 çekirdek (180 MHz)
  - 512 KB Flash bellek
  - 128 KB SRAM
  - FPU (Floating Point Unit) desteği

### Bellek
- **Flash Bellek:** W25Qxx serisi SPI Flash
  - Yüksek hızlı veri kayıt
  - Uçuş verilerinin kalıcı depolanması

---

## 📡 Sensörler ve Modüller

### 1. **BMI088** - 6-Eksen IMU
- **Özellikler:**
  - 3-eksen jiroskop (±2000°/s)
  - 3-eksen ivmeölçer (±24g)
  - Yüksek hassasiyet ve düşük gürültü
- **Kullanım:** Yönelim tespiti, rotasyon hızı ölçümü

### 2. **BME280** - Çevresel Sensör
- **Özellikler:**
  - Barometrik basınç sensörü
  - Sıcaklık sensörü
  - Nem sensörü
- **Kullanım:** İrtifa hesaplama, atmosferik veri toplama

### 3. **L86 GNSS Modülü**
- **Özellikler:**
  - GPS/GLONASS/Galileo desteği
  - Yüksek hassasiyetli konum belirleme
  - NMEA protokolü
- **Kullanım:** Konum takibi, hız ve rota bilgisi

### 4. **E22 LoRa Modülü**
- **Özellikler:**
  - Uzun menzilli kablosuz iletişim
  - 433/868/915 MHz frekans desteği
- **Kullanım:** Gerçek zamanlı telemetri veri aktarımı

---

## ✨ Özellikler

### 🎯 Uçuş Algoritması
- **Durum Makinesi Tabanlı Uçuş Aşaması Tespiti**
  - İniş öncesi (Pre-Landing)
  - Fırlatma (Launch)
  - Tırmanma (Ascent)
  - Apoje (Apogee)
  - İniş (Descent)
  - Karaya değme (Landing)
- Gerçek zamanlı uçuş aşaması izleme
- Akıllı apoje tespiti algoritması

### 📊 Sensör Füzyonu
- **Kalman Filtresi** implementasyonu
  - İrtifa tahmini için gelişmiş filtreleme
  - Sensör verilerinin birleştirilmesi
  - Gürültü azaltma
- **Quaternion tabanlı yönelim hesaplama**
  - Euler açılarından kaçınma (Gimbal Lock problemi)
  - Roll, Pitch, Yaw hesaplama
  - Sensör verilerinin füzyonu

### 💾 Veri Kayıt Sistemi
- **SD Kart/Flash Bellek Desteği**
  - FATFS dosya sistemi entegrasyonu
  - Yüksek hızlı veri yazma
  - CSV formatında veri kaydetme
- Tüm sensör verilerinin zaman damgalı kaydı

### 📦 Paket Yönetimi
- Optimize edilmiş veri paketleme
- Telemetri için veri formatı oluşturma
- CRC/Checksum doğrulama

### ⚡ Performans İzleme
- **DWT Profiler** entegrasyonu
  - Gerçek zamanlı kod performans analizi
  - Fonksiyon çalışma süresi ölçümü
  - Sistem kaynak kullanımı takibi

---

## 🚀 Kurulum

### Gereksinimler

#### Yazılım
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (v1.13.0 veya üzeri)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) (v6.9.0 veya üzeri)
- ARM GCC Toolchain (STM32CubeIDE ile birlikte gelir)

#### Donanım
- STM32F446RE Nucleo Board veya özel PCB
- ST-Link V2 Programlayıcı
- USB Kablosu (Mini-B veya Micro-B)

### Repository'yi Klonlama

```bash
git clone https://github.com/halilsrky/PayLord.git
cd PayLord
```

---

## 🔨 Derleme ve Yükleme

### STM32CubeIDE ile Derleme

1. **Projeyi Açma**
   ```
   File → Open Projects from File System
   PayLord klasörünü seçin
   ```

2. **Derleme**
   ```
   Project → Build All (Ctrl+B)
   ```

3. **Yükleme**
   ```
   Run → Debug (F11) veya Run (Ctrl+F11)
   ```

### Komut Satırı ile Derleme

```bash
# Debug build
make -C Debug

# Release build
make -C Release

# Temizleme
make -C Debug clean
```

### Flash Yükleme (ST-Link)

```bash
st-flash write Debug/PayLord.bin 0x8000000
```

---

## 📖 Kullanım

### İlk Başlatma

1. **Donanım Bağlantıları**
   - Tüm sensörlerin doğru pinlere bağlandığından emin olun
   - Güç kaynağını bağlayın (3.3V/5V)

2. **Konfigürasyon**
   - `Core/Inc/configuration.h` dosyasından sistem ayarlarını yapılandırın
   ```c
   #define SAMPLE_RATE_HZ 100      // Örnekleme frekansı
   #define TELEMETRY_RATE_HZ 10    // Telemetri gönderim frekansı
   #define ENABLE_SD_CARD 1        // SD kart kullanımı
   ```

3. **Sistem Testi**
   - Sistemi başlatın
   - Seri port üzerinden debug mesajlarını kontrol edin
   - Sensör verilerinin geldiğini doğrulayın

### Veri Okuma

#### Seri Port (UART)
```
Baud Rate: 115200
Data Bits: 8
Stop Bits: 1
Parity: None
```

#### Telemetri Paketi Formatı
Sistem, aşağıdaki verileri düzenli olarak gönderir:
- Zaman damgası
- GPS koordinatları
- İrtifa (barometrik ve hesaplanmış)
- Hız bileşenleri
- İvme değerleri (X, Y, Z)
- Açısal hız değerleri (Roll, Pitch, Yaw)
- Uçuş aşaması
- Sistem durumu

---

## 📁 Kod Yapısı

```
PayLord/
├── Core/
│   ├── Inc/                          # Header dosyaları
│   │   ├── main.h                    # Ana yapılandırma
│   │   ├── configuration.h           # Sistem konfigürasyonu
│   │   ├── bmi088.h                  # IMU sürücüsü
│   │   ├── bme280.h                  # Barometrik sensör
│   │   ├── l86_gnss.h                # GPS modülü
│   │   ├── e22_lib.h                 # LoRa modülü
│   │   ├── flight_algorithm.h        # Uçuş algoritması
│   │   ├── kalman.h                  # Kalman filtresi
│   │   ├── quaternion.h              # Quaternion matematik
│   │   ├── sensor_fusion.h           # Sensör füzyonu
│   │   ├── data_logger.h             # Veri kayıt sistemi
│   │   ├── packet.h                  # Paket yönetimi
│   │   └── w25_flash_memory.h        # Flash bellek sürücüsü
│   │
│   └── Src/                          # Kaynak dosyaları
│       ├── main.c                    # Ana program döngüsü
│       ├── bmi088.c                  # IMU implementasyonu
│       ├── bme280.c                  # Barometrik sensör
│       ├── l86_gnss.c                # GPS parser
│       ├── e22_lib.c                 # LoRa iletişim
│       ├── flight_algorithm.c        # Uçuş aşaması mantığı
│       ├── kalman.c                  # Kalman filtresi
│       ├── quaternion.c              # Quaternion hesaplamaları
│       ├── sensor_fusion.c           # Füzyon algoritması
│       ├── data_logger.c             # SD kart yazma
│       ├── packet.c                  # Veri paketleme
│       ├── w25_flash_memory.c        # Flash operasyonları
│       └── dwt_profiler.c            # Performans profiler
│
├── Drivers/                          # STM32 HAL sürücüleri
├── FATFS/                            # FAT dosya sistemi
├── Middlewares/                      # USB middleware
├── Debug/                            # Debug build çıktıları
├── PayLord.ioc                       # STM32CubeMX proje dosyası
└── README.md                         # Bu dosya
```

---

## 🎓 Teknik Detaylar

### Sensör Okuma Frekansları
- **IMU (BMI088):** 1000 Hz
- **Barometre (BME280):** 100 Hz
- **GPS (L86):** 10 Hz (NMEA mesajları)
- **Telemetri:** 10 Hz

### İletişim Protokolleri
- **I2C:** BME280 sensörü için
- **SPI:** BMI088, W25Qxx Flash için
- **UART:** GPS modülü, LoRa modülü, Debug çıktısı

### Interrupt Yönetimi
- Timer interrupt ile hassas zamanlama
- DMA kullanımı ile CPU yükü azaltma
- Priority tabanlı interrupt işleme

---

## 🧪 Test ve Doğrulama

### Birim Testleri
Her modül için ayrı test fonksiyonları mevcuttur:
```c
void test_bmi088(void);      // IMU testi
void test_bme280(void);      // Barometre testi
void test_gnss(void);        // GPS testi
void test_flash(void);       // Flash bellek testi
```

### Simülasyon
Uçuş algoritması simüle edilmiş verilerle test edilebilir:
```c
void simulate_flight_data(void);
```

---

## 🤝 Katkıda Bulunma

Bu proje, Aeronist Aerospace takımının TEKNOFEST yarışması için geliştirilmiştir. Önerileriniz ve katkılarınız için:

1. Repository'yi fork edin
2. Feature branch oluşturun (`git checkout -b feature/YeniOzellik`)
3. Değişikliklerinizi commit edin (`git commit -m 'Yeni özellik eklendi'`)
4. Branch'inizi push edin (`git push origin feature/YeniOzellik`)
5. Pull Request oluşturun

---

## 📝 Lisans

Bu proje [MIT Lisansı](LICENSE) altında lisanslanmıştır.

---

## 📞 İletişim

**Aeronist Aerospace Takımı**

- 🌐 GitHub: [@halilsrky](https://github.com/halilsrky)
- 📧 E-posta: [Takım e-posta adresi]
- 🚀 TEKNOFEST: [Takım sayfası]

---

## 🏆 Teşekkürler

- TEKNOFEST organizasyon komitesine
- STMicroelectronics topluluğuna
- Aeronist Aerospace takım üyelerine
- Danışman hocalarımıza

---

## 📚 Referanslar

- [STM32F446RE Datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [BME280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/)
- [TEKNOFEST Yarışma Kuralları](https://www.teknofest.org/)

---

**🚀 Başarılı uçuşlar dileriz! 🚀**

*"Gökyüzü limit değil, sadece başlangıç."*
