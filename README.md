# HydroSense: Estación de Monitoreo Ambiental Autónoma

**HydroSense** es un sistema embebido robusto y de bajo consumo basado en el microcontrolador **ESP32**, diseñado para la monitorización remota y autónoma de variables ambientales y agrícolas. El sistema recolecta datos de múltiples sensores, los almacena localmente en una tarjeta Micro SD y los transmite a través de la red celular mediante SMS.

El proyecto está optimizado para una operación prolongada con baterías gracias a un eficiente ciclo de sueño profundo (`deep sleep`), y ofrece modos de configuración avanzados a través de **Bluetooth** y un **servidor web integrado** para la descarga de datos y actualizaciones de firmware **OTA (Over-The-Air)**.

## ✨ Características Principales

* **Bajo Consumo de Energía:** Utiliza el modo `deep sleep` del ESP32 y MOSFETs para cortar la alimentación de los periféricos, maximizando la vida de la batería.
* **Doble Vía de Comunicación:** Envío de datos a través de SMS y configuración local mediante Bluetooth.
* **Servidor Web Integrado:** Permite descargar los logs de datos `.csv` y actualizar el firmware de forma inalámbrica (OTA) a través de un punto de acceso WiFi.
* **Almacenamiento Local Robusto:** Guarda todas las mediciones en una tarjeta Micro SD en archivos `.csv` semanales.
* **Configuración Flexible:** Todos los parámetros del sistema se gestionan a través de un archivo `config.json`, permitiendo ajustes sin necesidad de reprogramar.
* **Manejo Avanzado de Errores:** Sistema de estados que diferencia entre fallos no críticos, medios y críticos, con capacidad de enviar notificaciones de alerta y proteger el sistema.
* **Múltiples Sensores:** Integra una amplia gama de sensores para una monitorización completa:
    * Temperatura y humedad ambiental.
    * Temperatura y humedad del suelo a dos profundidades.
    * Radiación solar (Piranómetro).

## 🛠️ Componentes de Hardware

| Componente                | Descripción                                        | Conexión    |
| ------------------------- | -------------------------------------------------- | ----------- |
| **Microcontrolador** | ESP32 Dev Module                                   | -           |
| **Comunicación Celular** | Módulo SIM800L                                     | UART        |
| **Reloj en Tiempo Real** | RTC DS3231 (Alta precisión)                        | I²C         |
| **ADC de Precisión** | ADC ADS1115 (16-bit)                               | I²C         |
| **Sensor Ambiental** | SHT10 (Temperatura y Humedad)                      | Propietario |
| **Sensores de Suelo** | 2x Sensores de Humedad capacitivos                 | Analógico   |
| **Sensores de Temperatura**| 2x DS18B20 (Sumergibles)                           | OneWire     |
| **Sensor de Radiación** | Piranómetro con salida Modbus                      | RS485       |
| **Almacenamiento** | Módulo para Tarjeta Micro SD                       | SPI         |
| **Control de Energía** | 2x MOSFETs (para SIM800L y periféricos)            | GPIO        |

## 🕹️ Modos de Operación

El modo de operación se selecciona al arrancar el dispositivo manteniendo presionado el botón conectado al `PIN_BLUETOOTH_ACTIVADOR` (GPIO 12).

1. **Modo Normal (Pulsación < 5s o ninguna):**
   * El dispositivo despierta del `deep sleep`.
   * Lee todos los sensores.
   * Guarda los datos en la tarjeta SD.
   * Envía los datos por SMS.
   * Vuelve a `deep sleep` por el intervalo configurado.

2. **Modo de Configuración (Pulsación de 5s a 10s):**
   * El dispositivo activa el **Bluetooth** y espera una conexión.
   * Al conectar desde un celular o PC con una terminal Bluetooth (ej. Serial Bluetooth Terminal), se despliega un menú interactivo que permite:
     * Ver y editar todos los parámetros del sistema.
     * Realizar una calibración guiada de los sensores de humedad.
     * Sincronizar la hora del RTC.
     * Ejecutar pruebas de diagnóstico para cada componente.
     * Guardar la configuración permanentemente.

3. **Modo Servidor Web (Pulsación > 10s):**
   * El dispositivo crea un **Punto de Acceso WiFi** con el SSID `HydroSense-AP`.
   * Al conectarse a esta red, se puede acceder a una interfaz web desde el navegador (IP por defecto: `192.168.4.1`) que permite:
     * Listar, descargar y eliminar los archivos de log `.csv`.
     * **Actualizar el firmware** del dispositivo de forma inalámbrica (OTA).

## ⚙️ Configuración

La configuración del sistema se gestiona a través del archivo `config.json` en la raíz de la tarjeta Micro SD. Si el archivo no existe, el sistema usará valores por defecto.

```json
{
  "nombre_equipo": "HydroSense-P1",
  "numSerie": "SN-001",
  "sueloS30_min": 11500,
  "sueloS30_max": 24500,
  "sueloS15_min": 12000,
  "sueloS15_max": 25000,
  "intervalo_minutos": 15,
  "numero_SMSDatos": "+520000000000",
  "numero_SMSNotif": "+520000000000",
  "usar_modbus": true,
  "usar_sim800": true,
  "usar_leds_estado": true,
  "reintentos_envio_sms": 3,
  "tolerancia_adc": 250,
  "tempSuelo1_addr": "28:32:B0:87:00:2D:04:A4",
  "tempSuelo2_addr": "28:DE:62:87:00:03:79:35"
}
```

## 🚀 Instalación y Puesta en Marcha

Este proyecto está desarrollado utilizando **PlatformIO**.

1. **Clonar el repositorio:**
   ```bash
   git clone https://[URL-DEL-REPOSITORIO]
   cd [NOMBRE-DEL-REPOSITORIO]
   ```

2. **Abrir con PlatformIO:**
   Abre la carpeta del proyecto en VSCode con la extensión de PlatformIO.

3. **Instalar Librerías:**
   PlatformIO instalará automáticamente todas las dependencias listadas en el archivo `platformio.ini`.

4. **Particiones:**
   El sistema utiliza una tabla de particiones personalizada (`partitions.csv`) para habilitar las actualizaciones OTA. PlatformIO la gestionará automáticamente.

5. **Subir el código:**
   Conecta tu ESP32, selecciona el puerto correcto y sube el firmware.

6. **Preparar la SD:**
   Formatea una tarjeta Micro SD en FAT32 y, opcionalmente, copia el archivo `config.json` a la raíz para una configuración personalizada.

## 📚 Librerías Utilizadas

Las siguientes librerías son gestionadas por PlatformIO a través del archivo `platformio.ini`:

* DallasTemperature para sensores DS18B20.
* OneWire by Paul Stoffregen (dependencia de DallasTemperature).
* Adafruit ADS1X15 para el ADC de precisión.
* TinyGSM by Volodymyr Shymanskyy para el módem SIM800L.
* ArduinoJson by Benoit Blanchon para la gestión del config.json.
* SHT1x-ESP by "enjoyneering" para el sensor SHT10.
* RTClib by Adafruit para el RTC DS3231.
* ModbusMaster by "4-20ma" para la comunicación RS485.

## 📌 Pinout del Sistema

| Pin ESP32 | Componente                    | Función                     |
| --------- | ----------------------------- | ---------------------------- |
| 13        | Sensores de Temperatura (DS18B20) | Bus de datos OneWire         |
| 15        | RTC DS3231 / ADC ADS1115      | I²C SDA (Datos)             |
| 4         | RTC DS3231 / ADC ADS1115      | I²C SCL (Reloj)             |
| 16        | Sensor Ambiental (SHT-10)     | Data                        |
| 17        | Sensor Ambiental (SHT-10)     | Clock                       |
| 5         | Tarjeta Micro SD              | CS (Chip Select)            |
| 18        | Tarjeta Micro SD              | SCK (Reloj)                 |
| 23        | Tarjeta Micro SD              | MOSI (Master Out)           |
| 19        | Tarjeta Micro SD              | MISO (Master In)            |
| 21        | MOSFET Periféricos            | Control de Alimentación     |
| 33        | Transceptor RS485             | RX                          |
| 32        | Transceptor RS485             | TX                          |
| 27        | Transceptor RS485             | DE/RE (Control Dir.)        |
| 26        | Módulo SIM800L                | RXD                         |
| 25        | Módulo SIM800L                | TXD                         |
| 22        | MOSFET SIM800L                | Control de Alimentación     |
| 14        | LED Rojo                      | Indicador de Estado         |
| 2         | LED Verde                     | Indicador de Estado         |
| 12        | Botón de Modo                 | Activador de Modos          |
