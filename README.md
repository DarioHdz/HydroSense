# 🌱 HydroSense – Sistema de Monitoreo Ambiental con ESP32

**HydroSense** es un sistema embebido autónomo basado en ESP32 diseñado para monitoreo ambiental en campo. Realiza lecturas de temperatura, humedad relativa, humedad del suelo en dos profundidades y radiación solar. Integra almacenamiento local en microSD, envío de datos vía GSM, alertas por SMS, configuración en sitio vía Bluetooth y actualización remota vía WiFi (OTA).

---

## 🧩 Componentes del sistema

| Componente              | Descripción                                                  |
|-------------------------|--------------------------------------------------------------|
| **ESP32**               | Microcontrolador principal                                   |
| **SHT10**               | Sensor de temperatura y humedad relativa (3.3 V)             |
| **ADS1115**             | Conversor ADC de 16 bits para lectura de sensores de humedad |
| **2 sensores de humedad** | Lectura en S30 y S15 mediante ADC                            |
| **Piranómetro (RS485)** | Sensor de radiación solar                                     |
| **RTC DS3231**          | Reloj en tiempo real con I2C                                  |
| **SIM800L**             | Módulo GSM para envío HTTP y alertas SMS                      |
| **MicroSD**             | Registro de datos y configuración (`config.json`)             |
| **MOSFETs IRLZ44N**     | Control de GND de módulos de 5 V (SIM800L y demás)            |
| **LEDs de estado**      | Indicadores visuales de estado (verde y rojo)                |
| **Bluetooth**           | Configuración y calibración en campo                         |
| **TP4056 (USB-C)**      | Módulo de carga para batería 3.7 V, con RPROG de 3 kΩ (~400 mA) |
| **Panel solar 5 V 2 W** | Fuente auxiliar de energía para recarga solar                |

---

## 🔌 Distribución de pines

| Módulo / Función       | Pines ESP32          |
|------------------------|----------------------|
| **I2C (RTC / ADS1115)**| SDA: GPIO15, SCL: GPIO4 |
| **SHT10**              | Data: GPIO16, Clock: GPIO17 |
| **SIM800L**            | RX: GPIO26, TX: GPIO25, GND control: GPIO22 |
| **MicroSD**            | CS: GPIO5, SPI: GPIO18/23/19, GND control: GPIO21 |
| **RS485 (Piranómetro)**| RX: GPIO33, TX: GPIO32, DE/RE: GPIO27 |
| **Bluetooth**          | Activador: GPIO12     |
| **LED Verde / Rojo**   | GPIO13 / GPIO14       |
| **MOSFETs GND Control**| Módulos: GPIO21, SIM800L: GPIO22 |
| **Reservados**         | No usar: GPIO0–3 (D0–D3) |

---

## 🗂️ Archivo de configuración (`config.json`)

Este archivo se encuentra en la microSD y permite configurar el sistema sin recompilar. Ejemplo:

```json
{
  "nombre_equipo": "ESP32_NODE_01",
  "numSerie": "CDHS001",
  "sueloS30_min": 9004,
  "sueloS30_max": 18000,
  "sueloS15_min": 9500,
  "sueloS15_max": 17000,
  "intervalo_minutos": 30,
  "apn": "internet.itelcel.com",
  "usuario_apn": "webgprs",
  "contrasena_apn": "webgprs2002",
  "numero_SMS": "+521234567890",
  "usar_modbus": true,
  "usar_sim800": true,
  "usar_leds_estado": true,
  "timeout_red": 20000,
  "archivo_log": "log.csv",
  "reintentos_envio_sms": 3,
  "espera_entre_reintentos_sms_ms": 1000,
  "api_host": "miapi.ejemplo.com",
  "api_puerto": 80,
  "api_endpoint": "/api/registro"
}
```

---

## ⚙️ Funcionalidades clave

- 📏 Lectura de variables ambientales: temperatura, humedad relativa, humedad del suelo, radiación solar
- 💾 Registro local en microSD en formato CSV
- 📡 Envío de datos a servidor HTTP (JSON vía SIM800L)
- 📲 Alerta por SMS ante fallos (críticos, medios, no críticos)
- 🔧 Configuración por Bluetooth: calibración, APN, número SMS, nombre de equipo, API
- 🌙 Deep sleep con intervalo configurable para ahorro de energía
- ⚠️ Manejo de fallos con indicadores LED y estados diferenciados
- 🌐 **Actualización OTA por WiFi** desde navegador, sin necesidad de cable ni recompilación

---

## 🌐 Interfaz WiFi en campo

Cuando se mantiene presionado el botón >5 s al encender el dispositivo, este entra en **modo AP local** (`HydroSense-AP`).

Desde el navegador puedes:

- 📥 Descargar archivos `.csv` de la microSD
- 🗑️ Eliminar archivos innecesarios
- 🔄 Subir nuevo firmware (`.bin`) desde la ruta `/update`

---

## 🧰 Desarrollo y compilación

**Entorno**: [PlatformIO](https://platformio.org/)  
**Framework**: Arduino  
**Particiones personalizadas (OTA):**

```
# partitions.csv
# Name,     Type, SubType, Offset,   Size
nvs,        data, nvs,     0x9000,   0x4000
otadata,    data, ota,     0xd000,   0x2000
app0,       app,  ota_0,   0x10000,  0x1E0000
app1,       app,  ota_1,   0x1F0000, 0x1E0000
```

**Dependencias (platformio.ini):**

```ini
lib_deps = 
  beegee-tokyo/SHT1x-ESP@^1.0.2
  adafruit/RTClib@^2.1.4
  adafruit/Adafruit BusIO@^1.17.2
  vshymanskyy/TinyGSM@^0.12.0
  adafruit/Adafruit ADS1X15@^2.5.0
  4-20ma/ModbusMaster@^2.0.1
  bblanchon/ArduinoJson@^7.4.2
```

---

## 🔋 Energía y consumo

- Batería de 3.7 V recargable (lítium)
- Módulo TP4056 USB-C con RPROG de 3 kΩ (~400 mA)
- Panel solar 5 V / 2 W para recarga continua
- Control de consumo mediante MOSFETs IRLZ44N (apagan GND de módulos de 5 V)
- Deep sleep y activación selectiva de módulos según configuración

---

## 📍 Estado del proyecto

- ✅ Versión actual: **v2.6**
- ✅ OTA WiFi funcional
- ✅ Interfaz web lista para campo
- ✅ Registro y envío de datos estables

---

## 📘 Licencia

[MIT License](LICENSE)