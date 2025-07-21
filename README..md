## English

- auto_landing.py = servo + end mission 
- autonomous.py = servo only

### Overview
This Python script provides autonomous flight control for PX4-based drones with integrated servo motor control for payload release. The system uses MAVSDK for communication and supports waypoint-based missions with automatic return-to-launch functionality.

### Key Features Added

#### Servo Motor Control System
The script now includes comprehensive servo motor control for payload release operations:

**Main Functions:**
- `servo_release_payload()` - Controls the main payload release sequence
- `servo_test()` - Tests servo movement before flight operations
- `initialize_servo()` - Sets servo to secure position during startup
- `check_for_release_command()` - Monitors for manual release triggers

**Configuration Parameters:**
```python
SERVO_CHANNEL = 1          # AUX1 port (channels 1-8 available)
SERVO_RELEASE_PWM = 2000   # PWM value for release position
SERVO_SECURE_PWM = 1000    # PWM value for secure position
SERVO_HOLD_TIME = 1.0      # Time to hold release position
```

**Hardware Setup:**
1. Connect servo motor to AUX1 port on flight controller
2. Ensure proper power supply (5V from FC or separate BEC)
3. Configure PX4 parameter: `AUX_1_FUNCTION = Passthrough`

####  Waypoint-Based Mission Control
The mission system has been enhanced with structured waypoint management:

**New Mission Features:**
- `generate_target_location(max_waypoints)` - Generates specified number of waypoints
- `end_mission_safely()` - Handles clean mission completion with return-to-launch
- `execute_mission(drone, max_waypoints)` - Main mission control with waypoint tracking

**Mission Flow:**
1. **Initialization** - Servo setup and testing
2. **Takeoff** - Standard PX4 takeoff sequence
3. **Waypoint Navigation** - Sequential waypoint completion (1/10, 2/10, etc.)
4. **Payload Release** - Automatic release at specified waypoint (default: waypoint 3)
5. **Mission Completion** - Return to launch point
6. **Safe Landing** - Automated landing at launch coordinates

#### Safety Features
- **Servo Initialization** - Always starts in secure position
- **Pre-flight Testing** - Servo movement verification
- **Emergency Protocols** - Servo securing before emergency landing
- **Mission Boundaries** - Automatic mission completion after waypoint limit
- **Return-to-Launch** - Ensures drone returns to starting point

### Usage Instructions

#### Basic Configuration
```python
# Mission settings
max_waypoints = 10        # Number of waypoints to complete
flight_altitude = 3       # Flight altitude in meters
target_threshold = 0.5    # Waypoint accuracy threshold

# Payload release settings
release_waypoint = 3      # Which waypoint triggers payload release
```

#### Running the Mission
```bash
python drone_control.py
```

#### Expected Output
```
Drone connected!
Initializing servo channel 1 to secure position
Testing servo before takeoff...
Drone takeoff completed!
Offboard mode activated!

Waypoint 1/10
Gelen veri: dx=3.00 m, dy=0.00 m
...
Release waypoint reached!
Payload released!
...
Mission ending - returning to launch point...
Reached launch point!
Landing completed - Mission successful!
```

#### Customization Options
- **Change Mission Length:** Modify `max_waypoints` parameter
- **Adjust Release Timing:** Change `release_waypoint` number
- **Modify Flight Pattern:** Update `dx, dy` values in target generation
- **Configure Servo:** Adjust PWM values for your specific servo motor

---

## Türkçe Dokümantasyon

### Genel Bakış
Bu Python scripti, yük bırakma için entegre servo motor kontrolü olan PX4 tabanlı dronlar için otonom uçuş kontrolü sağlar. Sistem, iletişim için MAVSDK kullanır ve otomatik kalkış noktasına dönüş işlevi ile waypoint tabanlı görevleri destekler.

### Eklenen Temel Özellikler

#### Servo Motor Kontrol Sistemi
Script artık yük bırakma operasyonları için kapsamlı servo motor kontrolü içermektedir:

**Ana Fonksiyonlar:**
- `servo_release_payload()` - Ana yük bırakma sırasını kontrol eder
- `servo_test()` - Uçuş öncesi servo hareketini test eder
- `initialize_servo()` - Başlangıçta servoyu güvenli konuma ayarlar
- `check_for_release_command()` - Manuel bırakma tetikleyicilerini izler

**Konfigürasyon Parametreleri:**
```python
SERVO_CHANNEL = 1          # AUX1 portu (1-8 kanal mevcut)
SERVO_RELEASE_PWM = 2000   # Bırakma pozisyonu PWM değeri
SERVO_SECURE_PWM = 1000    # Güvenli pozisyon PWM değeri
SERVO_HOLD_TIME = 1.0      # Bırakma pozisyonunu tutma süresi
```

**Donanım Kurulumu:**
1. Servo motoru uçuş kontrolcüsündeki AUX1 portuna bağlayın
2. Uygun güç kaynağını sağlayın (FC'den 5V veya ayrı BEC)
3. PX4 parametresini yapılandırın: `AUX_1_FUNCTION = Passthrough`

#### Waypoint Tabanlı Görev Kontrolü
Görev sistemi yapılandırılmış waypoint yönetimi ile geliştirilmiştir:

**Yeni Görev Özellikleri:**
- `generate_target_location(max_waypoints)` - Belirtilen sayıda waypoint oluşturur
- `end_mission_safely()` - Kalkış noktasına dönüşle temiz görev tamamlamayı yönetir
- `execute_mission(drone, max_waypoints)` - Waypoint takibi ile ana görev kontrolü

**Görev Akışı:**
1. **Başlatma** - Servo kurulumu ve test
2. **Kalkış** - Standart PX4 kalkış sırası
3. **Waypoint Navigasyonu** - Sıralı waypoint tamamlama (1/10, 2/10, vb.)
4. **Yük Bırakma** - Belirtilen waypoint'te otomatik bırakma (varsayılan: 3. waypoint)
5. **Görev Tamamlama** - Kalkış noktasına geri dönüş
6. **Güvenli İniş** - Kalkış koordinatlarında otomatik iniş

#### Güvenlik Özellikleri
- **Servo Başlatma** - Her zaman güvenli konumda başlar
- **Uçuş Öncesi Test** - Servo hareket doğrulaması
- **Acil Protokoller** - Acil iniş öncesi servo güvenceye alma
- **Görev Sınırları** - Waypoint limitinden sonra otomatik görev tamamlama
- **Kalkış Noktasına Dönüş** - Dronun başlangıç noktasına dönmesini sağlar

### Kullanım Talimatları

#### Temel Konfigürasyon
```python
# Görev ayarları
max_waypoints = 10        # Tamamlanacak waypoint sayısı
flight_altitude = 3       # Metre cinsinden uçuş yüksekliği
target_threshold = 0.5    # Waypoint doğruluk eşiği

# Yük bırakma ayarları
release_waypoint = 3      # Hangi waypoint yük bırakmayı tetikler
```

#### Görevi Çalıştırma
```bash
python drone_control.py
```

#### Beklenen Çıktı
```
Drone connected!
Initializing servo channel 1 to secure position
Testing servo before takeoff...
Drone takeoff completed!
Offboard mode activated!

Waypoint 1/10
Gelen veri: dx=3.00 m, dy=0.00 m
...
Release waypoint reached!
Payload released!
...
Mission ending - returning to launch point...
Reached launch point!
Landing completed - Mission successful!
```

#### Özelleştirme Seçenekleri
- **Görev Uzunluğunu Değiştir:** `max_waypoints` parametresini değiştir
- **Bırakma Zamanlamasını Ayarla:** `release_waypoint` numarasını değiştir
- **Uçuş Desenini Değiştir:** Hedef oluşturmada `dx, dy` değerlerini güncelle
- **Servo Yapılandır:** Özel servo motorunuz için PWM değerlerini ayarlayın

---


## Troubleshooting / Sorun Giderme

### Common Issues / Yaygın Sorunlar

**English:**
- **Servo not responding:** Check AUX port wiring and PX4 parameter configuration
- **Mission not ending:** Verify waypoint threshold and GPS accuracy
- **Connection issues:** Ensure correct serial port and baud rate

**Türkçe:**
- **Servo yanıt vermiyor:** AUX port kablolama ve PX4 parametre yapılandırmasını kontrol edin
- **Görev bitmiyor:** Waypoint eşiği ve GPS doğruluğunu doğrulayın
- **Bağlantı sorunları:** Doğru seri port ve baud rate'i sağlayın