# Documentación de codigo HydroSense

Github ESP32: https://github.com/DarioHdz/HydroSense

# Código

# Lista de pines

## Pines no utilizados

Estos pines no son utilizados en el sistema debido a que pueden causar interferencias con el funcionamiento del microcontrolador(ESP32) como lo es el bloqueo de arranque, interferencia con la programación por USB o estados lógicos críticos

```c
  + NOT TO USE:
    - D0 // Pin de arranque
    - D1 // Usado para el puerto serie por defecto
    - D3 // Usado para la carga de firmware desde USB
```

## Pines DS3231 (Reloj de tiempo real RTC)

Pines utilizados para la comunicación con el modulo RTC utilizado

```c
+ DS3231 - 300 µA
    - SDA 15 // Transfiere datos entre el ESP32 y el DS3231
    - SCL 4 // Sincroniza el envio/recepcion de bits en I²C
```

## Pines SHT - 10 (Sensor digital de humedad y temperatura)

Comunicación con el sensor usado para la recolección de datos de humedad y temperatura en el ambiente

```c
+ SHT-10 - 1 mA
    - Data 16 // Linea de datos bidireccional
    - Clock 17 // Señal de reloj
```

## Pines MicroSD

Comunicación con el modulo microSD utilizado para la lectura y escritura de datos

```c
+ Micro SD - 100mA
    - CS 5 // Seleccciona la tarjeta SD cuando el ESP 32 necesita comunicar
    - SCK 18 // Señal de reloj que marca el ritmo de transmision SPI
    - MOSI 23 // Datos enviados desde el ESP32 a la microSD
    - MISO 19 // Datos enviados desde la microSD al ESP32
    - POWER 21 // Controla la alimentacion del modulo SD
```

## Pines ADS1115 (Analog to Digital Converter)

Comunicación con el ADC que se usa para leer los sensores de humedad en el suelo

```c
+ ADC sensores suelo
    - SDA 15 // Linea de datos bidireccional del bus I²C
    - SCL 4 // Linea de reloj del bus I²C
    - POWER 21 // Alimentacion del modulo ADS1115 controlada por el ESP32
```

## Pines RS485

Comunicación con el modulo RS485, permite leer el piranómetro (sensor de radiación solar) usando el protocolo Modbus RTU

```c
+ Pines RS485
    - RX 33 // Recepcion de datos desde el transceptor RS485
    - TX 32 // Envio de datos al transceptor RS485
    - DE/RE 27 // Control de direccion permite alternar entre transmitir y recibir
    - POWER 21 // Alimenta el modulo transceptor RS485
```

## Pines SIM800L

Comunicación con el modulo SIM800L, proporciona conectividad GSM/GRPS para el envió de datos a una API y el envió y recepción de SMS

```c
+ SIM800L
    - RXD 26 // Recepcion de datos desde el SIM800L
    - TXD 25 // Transmisison de datos hacia el SIM800L
    - POWER 22 // Control de elimentacion del modulo
```

## Pines LEDs

Comunicación con los leds para indicar posibles problemas con los módulos en el sistema

```c
+ LEDs de estado
    - Rojo 14 // Control de alimentacion
    - Verde 13 // Control de alimentacion
```

## Pin de botón físico

Botón que permite alternar entre modos (configuración por Bluetooth y modo de servidor)

```c
+ Bluetooth
    - Activador 12
```

# Librerías utilizadas

### DallasTemperature
- Controla sensores de temperatura **DS18B20** a través del protocolo **OneWire**.
- En HydroSense se usa para obtener la temperatura de los dos sensores de suelo.
- Depende de la librería `OneWire`.

### Adafruit_ADS1X15
- Controla módulos ADC de alta resolución **ADS1015** y **ADS1115** por bus I²C.
- En HydroSense se usa para leer las señales analógicas de los sensores de humedad de suelo con mayor precisión que el ADC interno del ESP32.

### BluetoothSerial
- Permite usar el **Bluetooth clásico (Serial over Bluetooth)** del ESP32.
- Se emplea para el modo de configuración inalámbrica, mostrando un menú interactivo en la terminal Bluetooth.

### HardwareSerial
- Maneja los puertos UART adicionales (físicos) del ESP32.
- Permite definir y usar `Serial1`, `Serial2` con pines RX/TX personalizados, lo que es crucial en HydroSense para comunicarse con el **SIM800L** y el **transceptor RS485** sin interferir con el puerto de programación.

### TinyGsmClient
- Proporciona una interfaz de alto nivel para controlar módems GSM/GPRS como el **SIM800L**.
- Abstrae el uso de comandos AT para registrarse en la red, enviar SMS y gestionar conexiones de datos.

### ModbusMaster
- Implementa el protocolo **Modbus RTU** en modo maestro.
- Se utiliza para la comunicación en serie (generalmente sobre **RS485**) con dispositivos industriales.
- En HydroSense, se emplea para leer los datos del piranómetro (sensor de radiación).

### ArduinoJson
- Una librería esencial para serializar (crear) y deserializar (leer) documentos **JSON**.
- Es muy eficiente en el uso de memoria, ideal para microcontroladores.
- Se usa para leer y escribir los parámetros del sistema en el archivo `config.json` de la tarjeta SD.

### SHT1x-ESP
- Librería específica para la comunicación con los sensores de temperatura y humedad de la familia **SHT1x**.
- En HydroSense, se utiliza para obtener la temperatura y humedad ambiental.

### WebServer
- Permite crear un servidor web HTTP en el ESP32.
- En este proyecto, se usa para crear el punto de acceso WiFi y la interfaz web que permite descargar los logs y actualizar el firmware **OTA (Over-the-Air)**.

### Arduino
- Es el framework principal que proporciona todas las funciones básicas del sistema, como `pinMode()`, `digitalWrite()`, `delay()`, etc.
- Es la base sobre la que se construye todo el programa.

### OneWire
- Implementa el protocolo de comunicación **1-Wire** de Dallas Semiconductor.
- Permite la comunicación con múltiples dispositivos en un solo cable de datos.
- Es una dependencia necesaria para la librería `DallasTemperature`.

### Update
- Librería del núcleo del ESP32 que gestiona las actualizaciones de firmware **OTA**.
- Proporciona las funciones para escribir el nuevo programa en la memoria flash y reiniciar el dispositivo.

### RTClib
- Facilita la comunicación con una gran variedad de módulos de Reloj en Tiempo Real (RTC).
- En HydroSense, se utiliza para controlar el RTC **DS3231**, manteniendo la fecha y hora exactas para los registros de datos.

### Wire
- Es la librería principal de Arduino/ESP32 para la comunicación a través del bus **I²C**.
- Es una dependencia utilizada por las librerías `RTClib` (para el DS3231) y `Adafruit_ADS1X15`.

### WiFi
- Librería del núcleo del ESP32 que maneja toda la funcionalidad WiFi.
- Se utiliza para crear el **Punto de Acceso (AP)** en el modo de servidor web.

### FS
- Proporciona una capa de abstracción para el sistema de archivos (FileSystem).
- Es la base que permite trabajar con diferentes tipos de almacenamiento como la tarjeta SD.

### SD
- Permite leer y escribir archivos en una **tarjeta Micro SD** a través del bus SPI.
- En HydroSense es fundamental para guardar el archivo de configuración (`config.json`) y los logs de datos semanales (`.csv`).

# Módulos y su función en el sistema

## Temperatura de Suelo (DS18B20)
Este bloque de código inicializa los objetos necesarios para la comunicación con los sensores de temperatura de suelo **DS18B20**. Se define el pin del bus de datos y se preparan los controladores de las librerías **OneWire** y **DallasTemperature**.

```c
// ==========================
// Temperatura suelo (DS18B20)
// ==========================

// Define el pin GPIO del ESP32 al que se conecta el bus de datos OneWire
constexpr int oneWireBus = 13;

// Objeto que gestiona el protocolo de comunicación OneWire en el pin definido
OneWire oneWire(oneWireBus);

// Objeto principal para interactuar con los sensores DS18B20,
// utilizando el objeto oneWire previamente creado
DallasTemperature sensors(&oneWire);
```

## RTC DS3231

El RTC DS3231 mantiene la fecha y hora incluso sin alimentación. El sistema usa estos valores para etiquetar las mediciones y generar nombres de archivos de log correctos.

```c
// ===========
// RTC DS3231
// ===========

// Pines I2C dedicados para el RTC (SDA = datos, SCL = reloj)
#define SDA_PIN 15
#define SCL_PIN 4

// Objeto de la librería RTClib para controlar el módulo
RTC_DS3231 rtc;
```

## Sensor de Humedad/Temperatura (SHT10)

Define los pines y el objeto para controlar el sensor SHT10, que mide la temperatura y humedad del ambiente. Este sensor utiliza un protocolo de 2 hilos propietario, no I²C.

```c
// =============
// Sensor SHT-10
// =============

// Pines de datos y reloj para el sensor SHT10
#define DATA_PIN 16
#define CLOCK_PIN 17

// Objeto para controlar el sensor, especificando los pines y
// el nivel de voltaje de operación (3.3V)
SHT1x sht1x(DATA_PIN, CLOCK_PIN, SHT1x::Voltage::DC_3_3v);
```

## Tarjeta Micro SD

Configuración para la tarjeta de memoria SD, donde se almacenan los logs de datos y el archivo de configuración del sistema.

```c
// ==========
// Micro SD
// ==========

// Define el pin Chip Select (CS) para la comunicación SPI con la tarjeta SD
#define SD_CS 5

// Objeto global de tipo 'File' que se usará para abrir, leer // y escribir en los archivos de la tarjeta SD
File logFile;
```

## ADC Externo (ADS1115)

Para obtener lecturas precisas de los sensores de humedad del suelo, se utiliza un convertidor analógico-digital (ADC) externo de alta resolución, el ADS1115.

```c
// ===================
// ADC sensores suelo
// ===================

// Objeto de la librería de Adafruit para controlar el ADC ADS1115 por I²C
Adafruit_ADS1115 ads;
```

## Comunicación Modbus (RS485)

Estas definiciones configuran la comunicación con el sensor de radiación, que utiliza el protocolo industrial Modbus RTU sobre una interfaz física RS485.

```c
// ===========
// Pines RS485
// ===========

// Pin para controlar la dirección de la comunicación en el transceptor RS485
// (DE: Driver Enable / RE: Receiver Enable)
#define RS485_DE_RE 27

// Pines del puerto UART2 para la comunicación serial con el transceptor
#define RS485_RX 33
#define RS485_TX 32

// Objeto que implementa la lógica de un maestro Modbus
ModbusMaster node;
```

## Módem Celular (SIM800L)

Este bloque configura el módem SIM800L, que proporciona conectividad celular para el envío de datos y notificaciones de alerta a través de SMS.

```c
// =========
// SIM800L
// =========

// Pines del puerto UART1 para la comunicación serial con el módem
#define MODEM_RX 26
#define MODEM_TX 25

// Crea una instancia de HardwareSerial (UART1) para el módem
HardwareSerial sim800(1);

// Objeto de alto nivel de la librería TinyGsm para controlar el módem
TinyGsm modem(sim800);

// Bandera para saber si el módem fue inicializado correctamente
bool sim800Inicializado = false;
```

## Control de Alimentación (MOSFETs)

Para maximizar la duración de la batería, se utilizan MOSFETs como interruptores controlados por el ESP32 para cortar completamente la alimentación de los módulos que más consumen cuando no están en uso.

```c
// ==================
// MOSFET de control
// ==================

// Pin para controlar la alimentación general de los sensores y módulos (SD, RS485, etc.)
#define POWER_CTRL_PIN 21

// Pin dedicado para controlar la alimentación del módem SIM800L
#define MODEM_PWR 22
```

## LEDs de Estado

Define los pines para los diodos LED que sirven como indicadores visuales del estado del sistema (OK, error, actividad, etc.).

```c
// ==============
// LEDs de estado
// ==============

// Pin para el LED verde, usualmente indica estado 'OK' o actividad
#define LED_VERDE 2

// Pin para el LED rojo, usualmente indica un error o fallo
#define LED_ROJO 14
```

## Conectividad Bluetooth

Configuración para el modo de mantenimiento y diagnóstico a través de Bluetooth.

```c
// ==========
// Bluetooth
// ==========

// Pin conectado a un botón físico. Se lee al arrancar para decidir
// si entrar en modo normal, Bluetooth o Servidor Web.
#define PIN_BLUETOOTH_ACTIVADOR 12

// Bandera para indicar si el modo de configuración Bluetooth está activo
bool bluetoothActivo = false;

// Objeto para manejar la comunicación por Bluetooth Serial (SPP)
BluetoothSerial SerialBT;
```

## Servidor Web (OTA y Logs)

Parámetros para el modo de servidor web, que se activa para descargar logs y realizar actualizaciones de firmware de forma inalámbrica (OTA).

```c
// ==========
// WebServer
// ==========

// Nombre (SSID) de la red WiFi que creará el dispositivo
auto ssid_ap = "HydroSense-AP";

// Contraseña para la red WiFi
auto password_ap = "hydrosense";

// Objeto que gestionará el servidor web en el puerto 80 (HTTP)
WebServer servidorWeb(80);
```

## Variables Globales de Control

Este conjunto de variables y definiciones gestiona el estado general, los modos de operación y la lógica de la máquina de estados del sistema.

```c
// ==================================
// Variables para control del sistema
// ==================================

// Define la versión del firmware para fácil identificación
#define FIRMWARE_VERSION "v2.7"

// Enumeración que define los posibles estados de salud del sistema
enum EstadoSistema {
  ESTADO_OK,
  FALLO_CRITICO,
  FALLO_MEDIO,
  FALLO_NO_CRITICO
};

// Variable que almacena el estado actual del sistema
EstadoSistema estadoActual;

// Atributo RTC_DATA_ATTR: estas variables se guardan en la memoria RTC
// y su valor persiste durante el ciclo de deep sleep.
RTC_DATA_ATTR uint8_t ciclos = 0; // Contador de ciclos de despierto/dormido
RTC_DATA_ATTR EstadoSistema ultimoEstado = ESTADO_OK; // Guarda el último estado antes de dormir

// Banderas para controlar el modo de operación principal
bool modoBluetooth = false;
bool modoServidorWeb = false;
```
## Estructura de Configuración

Una única estructura de C++ (struct) agrupa todos los parámetros configurables del sistema. Al inicio, estos valores se cargan desde un archivo config.json en la tarjeta SD a esta estructura en memoria RAM.


```c
// ========================
// Configuracion del sistema
// ========================

// Define un tipo de dato que agrupa todas las configuraciones
struct ConfiguracionSistema {
  int espera_entre_reintentos_sms_ms; // Tiempo de espera entre reintentos de SMS
  uint8_t tempSuelo1_addr[8];         // Dirección única del sensor de temp. 1
  uint8_t tempSuelo2_addr[8];         // Dirección única del sensor de temp. 2
  int reintentos_envio_sms;           // Máximo de reintentos para enviar SMS
  char numero_SMSDatos[20];           // Teléfono para enviar datos CSV
  char numero_SMSNotif[20];           // Teléfono para enviar notificaciones de error
  char nombre_equipo[32];             // Nombre identificador del equipo
  int intervalo_minutos;              // Minutos que duerme el equipo entre mediciones
  bool usar_leds_estado;              // Activa o desactiva los LEDs de estado
  uint16_t timeout_red;               // Tiempo máximo para esperar conexión a la red celular
  char archivo_log[32];               // Nombre base del archivo de log
  int tolerancia_adc;                 // Margen de error para lecturas de humedad saturada
  char numSerie[16];                  // Número de serie del equipo
  int sueloS30_min;                   // Valor ADC para S30 100% húmedo
  int sueloS30_max;                   // Valor ADC para S30 0% húmedo
  int sueloS15_min;                   // Valor ADC para S15 100% húmedo
  int sueloS15_max;                   // Valor ADC para S15 0% húmedo
  bool usar_modbus;                   // Habilita/deshabilita el sensor Modbus
  bool usar_sim800;                   // Habilita/deshabilita el módem SIM800L
};

// Crea una instancia global de la estructura que contendrá
// la configuración activa del sistema
ConfiguracionSistema config;
```

# Prototipo de funciones

| **Función** | **Descripción breve** | **Rol en el sistema** |
| -------------------------------- | --------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `leerPromedioADC()`              | Lee un canal del ADC varias veces y calcula el promedio.                                  | Filtrar y estabilizar las lecturas de los sensores analógicos de humedad del suelo.               |
| `actualizarEstado()`             | Cambia el estado general del sistema y puede enviar alertas.                              | Gestionar la máquina de estados para el control de errores y el reporte de estado.                |
| `editarCampoConfig()`            | Edita un parámetro de configuración de tipo texto (char*).                                | Función de ayuda para el menú Bluetooth, usada para cambiar nombres, números, etc.                |
| `editarDireccionSensor()`        | Edita una dirección de 8 bytes de un sensor OneWire.                                      | Función específica del menú Bluetooth para configurar las direcciones de los sensores DS18B20.     |
| `editarCampoNumConfig()`         | Edita un parámetro de configuración de tipo numérico (int).                               | Función de ayuda para el menú Bluetooth, usada para cambiar intervalos, reintentos, etc.          |
| `parpadearLed()`                 | Hace parpadear un LED un número determinado de veces.                                     | Proporcionar feedback visual simple al usuario para indicar estados o acciones.                   |
| `calcularPorcentajeHumedad()`    | Convierte un valor crudo del ADC a un porcentaje de humedad.                              | Procesar los datos de los sensores de suelo utilizando los valores de calibración guardados.      |
| `alternarCampoBool()`            | Cambia el valor de un parámetro de configuración booleano (true/false).                   | Función de ayuda para el menú Bluetooth, para activar/desactivar módulos como Modbus o SIM800L.   |
| `enviarSMS()`                    | Gestiona el envío de un mensaje SMS, incluyendo reintentos en caso de fallo.              | Encargada de la comunicación a bajo nivel con el módem SIM800L para enviar datos y alertas.       |
| `getTemp()`                      | Obtiene la temperatura de una dirección específica de un sensor DS18B20.                  | Simplificar la lectura de temperatura y manejar posibles errores de desconexión del sensor.       |
| `parpadearAmbosLeds()`           | Hace parpadear los LEDs rojo y verde simultáneamente.                                     | Dar una señal visual distintiva, usada para indicar los umbrales de tiempo al detectar el modo.   |
| `enviarDatos()`                  | Orquesta el proceso de envío de los datos CSV por SMS.                                    | Función de alto nivel llamada en el `loop()` principal para la transmisión remota de datos.       |
| `guardarEnSD()`                  | Guarda una línea de texto en el archivo de log semanal en la tarjeta SD.                  | Asegurar la persistencia de los datos de forma local en la tarjeta de memoria.                   |
| `fechaActual()`                  | Formatea un objeto `DateTime` a un string estándar "YYYY/MM/DD HH:MM:SS".                 | Crear timestamps consistentes para los logs y los mensajes de estado.                             |
| `debugPrint()`                   | Imprime mensajes de depuración tanto en el puerto Serie como en Bluetooth.                | Centralizar todos los mensajes de estado del sistema para facilitar la depuración en PC o en campo. |
| `configuracionRapidaDespliegue()`| Inicia un asistente guiado en el menú Bluetooth para configurar el equipo.                | Simplificar y agilizar la puesta en marcha inicial del dispositivo en campo.                      |
| `escanearDispositivosOneWire()`  | Busca y muestra las direcciones de todos los dispositivos en el bus OneWire.              | Herramienta de diagnóstico en el menú de pruebas para identificar los sensores de temperatura.     |
| `sincronizarHoraPorBluetooth()`  | Permite al usuario ajustar la hora del RTC manualmente vía Bluetooth.                     | Utilidad del menú de configuración para asegurar que los timestamps sean correctos.               |
| `mostrarConfiguracionActual()`   | Imprime todos los parámetros de configuración actuales en la terminal Bluetooth.          | Permitir al usuario verificar rápidamente la configuración completa del equipo.                   |
| `probarSensoresTemperatura()`    | Ejecuta una prueba de lectura de los sensores de temperatura del suelo.                   | Herramienta de diagnóstico en el menú de pruebas para verificar el funcionamiento de los DS18B20. |
| `obtenerNombreLogSemanal()`      | Genera el nombre del archivo de log (`.csv`) basado en la fecha de inicio de la semana.   | Gestionar la organización de los datos en archivos de log semanales.                              |
| `menuCalibracionSensores()`      | Muestra el submenú para las opciones de calibración de sensores.                          | Organizar las funciones de calibración dentro del menú principal de Bluetooth.                    |
| `guardarConfigEnArchivo()`       | Guarda la configuración actual (struct `config`) en el archivo `config.json` en la SD.    | Hacer permanentes los cambios de configuración para que persistan tras un reinicio.               |
| `probarSensoresHumedad()`        | Ejecuta una prueba de lectura de los sensores de humedad del suelo.                       | Herramienta de diagnóstico en el menú de pruebas para verificar el ADC y los sensores.            |
| `calibrarSensoresSuelo()`        | Guía al usuario en el proceso interactivo de calibración de humedad.                      | Lógica principal para establecer los puntos de referencia de suelo seco y mojado.                 |
| `menuBluetoothGeneral()`         | Muestra el menú principal del modo de configuración Bluetooth.                            | Punto de entrada y bucle principal de toda la interfaz de configuración inalámbrica.              |
| `leerLineaBluetooth()`           | Lee una línea completa de texto enviada por el usuario vía Bluetooth.                     | Función de ayuda para obtener la entrada del usuario en los menús interactivos.                   |
| `editarConfiguracion()`          | Muestra el submenú para editar todos los parámetros del sistema.                          | Organizar las múltiples opciones de edición dentro del menú principal de Bluetooth.               |
| `cargarConfiguracion()`          | Lee el archivo `config.json` de la SD y carga los valores en el struct `config`.          | Inicializar el sistema con los parámetros guardados en lugar de valores por defecto.              |
| `inspectorComandosAT()`          | Permite enviar comandos AT directamente al módem SIM800L y ver la respuesta.              | Herramienta de depuración avanzada para diagnosticar problemas de conectividad celular.           |
| `leerRadiacionModbus()`          | Lee los registros del sensor de radiación a través de Modbus.                             | Encapsular la comunicación específica para obtener el valor de radiación.                         |
| `leerEnteroBluetooth()`          | Lee una línea de texto por Bluetooth y la convierte a un número entero.                   | Función de ayuda para obtener entradas numéricas del usuario en los menús.                        |
| `iniciarServidorWeb()`           | Configura el punto de acceso WiFi y el servidor web para OTA y descarga de logs.          | Inicializar todo lo necesario para el modo de operación de Servidor Web.                          |
| `detectarModoInicio()`           | Comprueba un pin al arrancar para decidir en qué modo de operación iniciar.               | Lógica de arranque crucial que determina si el equipo entra en modo normal, Bluetooth o WiFi.     |
| `probarEnvioNotif()`             | Envía un SMS de prueba al número de notificaciones.                                       | Herramienta de diagnóstico para verificar que las alertas de SMS funcionan.                       |
| `probarEnvioDatos()`             | Envía un SMS de prueba con datos simulados al número de datos.                            | Herramienta de diagnóstico para verificar que el envío de datos por SMS funciona.                 |
| `verificarModbus()`              | Realiza una lectura simple para confirmar la comunicación con el esclavo Modbus.          | Comprobar la conexión física y la configuración de Modbus durante la inicialización.              |
| `iniciarSIM800L()`               | Enciende, inicializa y conecta el módem SIM800L a la red celular.                         | Función de inicialización crítica para habilitar toda la comunicación por SMS.                    |
| `iniciarADS1115()`               | Inicializa la comunicación I²C con el ADC externo ADS1115.                                | Función de inicialización para el convertidor analógico-digital.                                  |
| `iniciarModbus()`                | Inicializa el puerto serie y el controlador para la comunicación Modbus/RS485.            | Función de inicialización para el sensor de radiación.                                            |
| `menuPruebas()`                  | Muestra el submenú de pruebas de hardware y comunicación.                                 | Organizar todas las herramientas de diagnóstico en una sección del menú Bluetooth.                |
| `iniciarRTC()`                   | Inicializa la comunicación I²C con el reloj en tiempo real DS3231.                        | Función de inicialización para asegurar que el sistema tenga una referencia de tiempo válida.     |
| `iniciarSD()`                    | Inicializa la comunicación SPI con la tarjeta Micro SD.                                   | Función de inicialización crítica para el almacenamiento local de datos y configuración.          |

# Funciones

## setup()
Función de configuración inicial que se ejecuta una sola vez cada vez que el sistema se enciende o se reinicia.

```c
void setup() {
  // Espera inicial de 5 segundos para dar tiempo a que el sistema se estabilice
  delay(5000);

  // --- Configuración de pines de entrada/salida ---
  // Pin para activar Bluetooth, con resistencia pull-up interna
  pinMode(PIN_BLUETOOTH_ACTIVADOR, INPUT_PULLUP);
  // Pines para controlar alimentación de módulos externos (MOSFETs)
  pinMode(POWER_CTRL_PIN, OUTPUT);
  pinMode(MODEM_PWR, OUTPUT);
  // Pines para los LEDs indicadores
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  // Apaga los LEDs al inicio
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_ROJO, LOW);

  // --- Inicialización de buses de comunicación ---
  // Define pines SDA y SCL personalizados para el bus I2C
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin(); // Inicia la comunicación I2C
  // Inicializa la comunicación serie para depuración por USB
  Serial.begin(115200);
  delay(2000); // Pequeña espera para que el puerto serie esté listo

  // --- Secuencia de arranque de periféricos críticos ---
  // Enciende la alimentación principal para los sensores y la SD
  digitalWrite(POWER_CTRL_PIN, HIGH);
  delay(500); // Espera a que los módulos se estabilicen
  // Si un módulo crítico falla en inicializar, la ejecución se detiene
  if (!iniciarSD()) return;
  if (!cargarConfiguracion()) return; // Carga la configuración desde la SD
  if (!iniciarADS1115()) return;
  if (!iniciarRTC()) return;

  // --- Detección del modo de operación ---
  // Comprueba si el botón de modo está presionado al arrancar
  detectarModoInicio();

  // Si se detectó el modo Servidor Web, lo inicia y detiene el setup normal
  if (modoServidorWeb) {
    debugPrint("🌐 Iniciando modo servidor web para descarga...");
    iniciarServidorWeb();
    return;
  }

  // Si se detectó el modo Bluetooth, lo inicia y detiene el setup normal
  if (modoBluetooth) {
    debugPrint("🔧 Iniciando modo Bluetooth...");
    bluetoothActivo = true;
    SerialBT.begin(config.nombre_equipo); // Inicia Bluetooth con el nombre del equipo
    menuBluetoothGeneral(); // Lanza el menú interactivo
    return;
  }

  // --- Inicio en modo de operación normal ---
  ciclos++; // Incrementa el contador de ciclos (persiste en deep sleep)
  debugPrint("🔁 Ciclo #" + String(ciclos));
  debugPrint("📦 Firmware: " + String(FIRMWARE_VERSION));
  debugPrint("🔧 Equipo: " + String(config.nombre_equipo));
  debugPrint("⚙️ Iniciando sistema...");

  // Inicializa el bus Modbus solo si está activado en la configuración
  if (config.usar_modbus && !iniciarModbus()) return;

  // Si todo salió bien, establece el estado del sistema como OK
  actualizarEstado(ESTADO_OK);
}
```

## loop()
Es el bucle principal del sistema. En modo normal, este código se ejecuta una sola vez gracias al deep sleep al final. En modo servidor web, se ejecuta continuamente para atender las peticiones de los clientes.

```c
void loop() {
  // Si el sistema está en modo Servidor Web, solo se dedica a atender clientes
  if (modoServidorWeb) {
    servidorWeb.handleClient();
    return;
  }

  // ======================================================
  // === INICIO DEL CICLO DE OPERACIÓN NORMAL (SOLO 1 VEZ) ===
  // ======================================================

  // Obtiene la fecha y hora actual del RTC
  const DateTime now = rtc.now();

  // --- Fase 1: Lectura de todos los sensores ---
  debugPrint("⏳ Leyendo sensores...");
  // Lee temperatura y humedad ambiental (SHT10)
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  if (isnan(tempC) || isnan(humidity)) { // Comprueba si la lectura fue válida
    tempC = -99.9;
    humidity = -99.9;
  }

  // Lee los valores crudos de los sensores de humedad del suelo (ADS1115)
  int suelo_ads_S30 = leerPromedioADC(0, 5, 50, false);
  int suelo_ads_S15 = leerPromedioADC(1, 5, 50, false);
  // Convierte los valores crudos a porcentaje
  int hS30 = calcularPorcentajeHumedad(suelo_ads_S30, config.sueloS30_max, config.sueloS30_min);
  int hS15 = calcularPorcentajeHumedad(suelo_ads_S15, config.sueloS15_max, config.sueloS15_min);

  // Formatea las lecturas de humedad para el reporte (maneja casos de saturación)
  String strS30 = (hS30 == -1) ? "SAT-H" : (hS30 == 101) ? "SAT-S" : String(hS30);
  String strS15 = (hS15 == -1) ? "SAT-H" : (hS15 == 101) ? "SAT-S" : String(hS15);

  // Lee el sensor de radiación (Modbus) si está habilitado
  int radiacion = config.usar_modbus ? leerRadiacionModbus() : -1;
  String radiacionStr = (radiacion < 0) ? "NA" : String(radiacion);

  // Lee los sensores de temperatura del suelo (DS18B20)
  sensors.requestTemperatures();
  float tempS1 = getTemp(config.tempSuelo1_addr);
  float tempS2 = getTemp(config.tempSuelo2_addr);

  // --- Fase 2: Formateo de datos ---
  // Construye la cadena de texto en formato CSV con todos los datos recolectados
  const String lineaCSV = String(config.nombre_equipo) + "," +
                          fechaActual(now) + "," +
                          String(tempC, 1) + "," +
                          String(humidity, 1) + "," +
                          strS30 + "," +
                          String(tempS1, 1) + "," +
                          String(suelo_ads_S30) + "," +
                          strS15 + "," +
                          String(tempS2, 1) + "," +
                          String(suelo_ads_S15) + "," +
                          radiacionStr;

  debugPrint("📊 Datos CSV a enviar: " + lineaCSV);

  // --- Fase 3: Almacenamiento y Transmisión ---
  // Guarda la línea de datos en la tarjeta SD
  guardarEnSD(lineaCSV);
  // Envía la línea de datos por SMS
  enviarDatos(lineaCSV);

  // Parpadea el LED verde para indicar que el ciclo fue exitoso
  parpadearLed(LED_VERDE, 3, false);
  delay(500);

  // --- Fase 4: Ahorro de energía y Deep Sleep ---
  // Apaga la alimentación del módem y de los sensores
  digitalWrite(MODEM_PWR, LOW);
  digitalWrite(POWER_CTRL_PIN, LOW);
  WiFi.disconnect(true); // Se asegura de que el WiFi esté apagado

  // Configura el temporizador para despertar al ESP32
  const uint64_t tiempo_sleep_us = static_cast<uint64_t>(config.intervalo_minutos) * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(tiempo_sleep_us);

  debugPrint("💤 Entrando en sleep profundo por " + String(config.intervalo_minutos) + " minutos...");
  delay(100);
  ultimoEstado = estadoActual; // Guarda el estado actual antes de dormir

  // Pone el microcontrolador en modo de ultra bajo consumo
  esp_deep_sleep_start();
}
```

## editarConfiguracion()
Muestra un submenú que permite al usuario modificar los parámetros del sistema de forma individual, como el nombre, los números de teléfono, el intervalo de medición, etc.

```c
void editarConfiguracion() {
  // Bucle que muestra el menú de edición hasta que el usuario elija salir (opción 0)
  while (true) {
    // Imprime las opciones del menú en la terminal Bluetooth
    SerialBT.println("\n⚙️ === EDITAR CONFIGURACIÓN ===");
    SerialBT.println("1️⃣ Cambiar nombre del equipo");
    SerialBT.println("2️⃣ Cambiar intervalo de medición (min)");
    SerialBT.println("3️⃣ Configurar número de teléfono para DATOS 🌐");
    SerialBT.println("4️⃣ Configurar número de teléfono para NOTIFICACIONES 🔔");
    SerialBT.println("5️⃣ Configurar reintentos de SMS");
    SerialBT.println("6️⃣ Activar/Desactivar SIM800L");
    SerialBT.println("7️⃣ Activar/Desactivar Modbus");
    SerialBT.println("8️⃣ Activar/Desactivar LEDs de estado");
    SerialBT.println("9️⃣ Cambiar Tolerancia ADC");
    SerialBT.println("1️⃣0️⃣ Editar dirección Sensor Temp 1");
    SerialBT.println("1️⃣1️⃣ Editar dirección Sensor Temp 2");
    SerialBT.println("0️⃣ Volver al menú principal 🔙");
    SerialBT.print("🔸 Selecciona una opción: ");

    // Espera y lee la opción seleccionada por el usuario
    String opcion = leerLineaBluetooth();

    // Llama a la función auxiliar correspondiente según la opción del usuario
    if (opcion == "1") {
      // Edita un campo de texto
      editarCampoConfig("nombre del equipo", config.nombre_equipo, sizeof(config.nombre_equipo));
    } else if (opcion == "2") {
      // Edita un campo numérico
      editarCampoNumConfig("intervalo de medición (min)", &config.intervalo_minutos);
    } else if (opcion == "3") {
      editarCampoConfig("número SMS DATOS", config.numero_SMSDatos, sizeof(config.numero_SMSDatos));
    } else if (opcion == "4") {
      editarCampoConfig("número SMS NOTIFICACIONES", config.numero_SMSNotif, sizeof(config.numero_SMSNotif));
    } else if (opcion == "5") {
      editarCampoNumConfig("reintentos de SMS", &config.reintentos_envio_sms);
    } else if (opcion == "6") {
      // Alterna un valor booleano (true/false)
      alternarCampoBool("SIM800L", &config.usar_sim800);
    } else if (opcion == "7") {
      alternarCampoBool("Modbus", &config.usar_modbus);
    } else if (opcion == "8") {
      alternarCampoBool("LEDs de estado", &config.usar_leds_estado);
    } else if (opcion == "9") {
      editarCampoNumConfig("Tolerancia ADC", &config.tolerancia_adc);
    } else if (opcion == "10") {
      // Edita la dirección de un sensor
      editarDireccionSensor("Sensor 1", config.tempSuelo1_addr);
    } else if (opcion == "11") {
      editarDireccionSensor("Sensor 2", config.tempSuelo2_addr);
    } else if (opcion == "0") {
      // Opción para salir del menú
      SerialBT.println("🔙 Volviendo al menú principal...");
      break; // Rompe el bucle while(true)
    } else {
      // Maneja el caso de una entrada no válida
      SerialBT.println("❌ Opción inválida. Intenta nuevamente.");
    }
  }
}
```

## enviarDatos()
Función de alto nivel que gestiona el envío de la cadena de datos CSV por SMS. Se encarga de verificar si el módem está listo y de registrar un error si el envío falla.

```c
void enviarDatos(const String& datosCSV) {
  // Comprueba si el módem ya fue inicializado previamente
  if (!sim800Inicializado) {
    debugPrint("📶 Módulo SIM apagado. Intentando iniciar...");
    // Si no lo está, intenta inicializarlo
    sim800Inicializado = iniciarSIM800L();
  }

  // Si después del intento de inicio, el módem sigue sin estar listo...
  if (!sim800Inicializado) {
    debugPrint("❌ No se pudo inicializar el módulo SIM para el envío de datos.");
    // Actualiza el estado del sistema a un fallo no crítico
    actualizarEstado(FALLO_NO_CRITICO, "SIM800L", "Fallo de inicio en envío");
  } else {
    // Si el módem está listo, intenta enviar el SMS con los datos
    if (!enviarSMS(config.numero_SMSDatos, datosCSV)) {
      // Si la función enviarSMS devuelve 'false' (fallo), actualiza el estado
      actualizarEstado(FALLO_NO_CRITICO, "SMS_DATOS", "Fallo en envío de datos");
    }
  }
}
```

## editarDireccionSensor()
Función auxiliar del menú para editar la dirección única de 8 bytes de un sensor OneWire (DS18B20). Utiliza sscanf para parsear el formato XX:XX:XX... ingresado por el usuario.


```c
void editarDireccionSensor(const char* nombreSensor, uint8_t* destino) {
  // Muestra las instrucciones al usuario
  SerialBT.println("✏️ Ingresa la nueva direccion para " + String(nombreSensor));
  SerialBT.println("   Formato: XX:XX:XX:XX:XX:XX:XX:XX");

  // Lee la dirección que el usuario escribe en la terminal Bluetooth
  String entrada = leerLineaBluetooth();
  entrada.trim(); // Elimina espacios en blanco al inicio o al final

  // Usa sscanf para parsear el string y convertir cada par de caracteres
  // hexadecimales (%hhx) en un byte y guardarlo en el array 'destino'.
  int n_scanned = sscanf(entrada.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                         &destino[0], &destino[1], &destino[2], &destino[3],
                         &destino[4], &destino[5], &destino[6], &destino[7]);

  // sscanf devuelve el número de elementos que pudo parsear correctamente.
  // Si pudo parsear 8 elementos, el formato es correcto.
  if (n_scanned == 8) {
    SerialBT.println("✅ Dirección actualizada correctamente.");
  } else {
    // Si no, el formato de entrada era incorrecto.
    SerialBT.println("❌ Error: Formato incorrecto. No se guardaron los cambios.");
  }
}
```

## escanearDispositivosOneWire()
Herramienta de diagnóstico que busca todos los dispositivos conectados al bus OneWire y muestra sus direcciones únicas en la terminal. Es muy útil para identificar los sensores de temperatura DS18B20 y verificar su conexión.


```c
void escanearDispositivosOneWire() {
  byte addr[8]; // Array de 8 bytes para almacenar la dirección del dispositivo encontrado

  SerialBT.println("\n🔍 Escaneando bus OneWire...");

  // Inicia la búsqueda. oneWire.search() devuelve 'true' si encuentra un dispositivo.
  if (oneWire.search(addr)) {
    int count = 0; // Contador para los dispositivos encontrados
    // Bucle 'do-while' para seguir buscando hasta que no se encuentren más dispositivos
    do {
      count++;
      SerialBT.print("  Dispositivo " + String(count) + ": ");

      // Itera a través de los 8 bytes de la dirección para imprimirla
      for (int i = 0; i < 8; i++) {
        if (addr[i] < 16) { // Si el valor hexadecimal es menor a 0x10...
          SerialBT.print("0"); // ...agrega un cero a la izquierda para el formato.
        }
        SerialBT.print(addr[i], HEX); // Imprime el byte en formato hexadecimal
        if (i < 7) {
          SerialBT.print(":"); // Agrega los dos puntos entre bytes
        }
      }
      SerialBT.println();

      // El primer byte de la dirección (family code) identifica el tipo de chip.
      // 0x28 corresponde a la familia de sensores DS18B20.
      if (addr[0] == 0x28) {
        SerialBT.println("    -> (Sensor de temperatura DS18B20)");
      }

    } while (oneWire.search(addr)); // Llama a search() de nuevo para encontrar el siguiente

    oneWire.reset_search(); // Reinicia el algoritmo de búsqueda de la librería

  } else {
    // Si la primera llamada a search() no encontró nada
    SerialBT.println("❌ No se encontraron dispositivos en el bus OneWire.");
    oneWire.reset_search();
  }
}
```

## inspectorComandosAT()
Proporciona una terminal interactiva para enviar comandos AT directamente al módem SIM800L y ver su respuesta cruda. Es una herramienta de depuración avanzada para diagnosticar problemas de conectividad.

```c
void inspectorComandosAT() {
  // Imprime las instrucciones de uso en la terminal
  SerialBT.println("\n🕵️ === Inspector de Comandos AT ===");
  SerialBT.println("Escribe un comando AT y presiona Enter para enviarlo al módem.");
  SerialBT.println("Escribe 'SALIR' para volver al menú principal.");
  SerialBT.println("----------------------------------------");

  // Se asegura de que el módem esté encendido antes de continuar
  if (!sim800Inicializado) {
    SerialBT.println("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L();
  }
  if (!sim800Inicializado) {
    SerialBT.println("❌ No se pudo inicializar el módulo SIM. Volviendo al menú.");
    return; // Sale de la función si el módem no se pudo iniciar
  }

  // Bucle principal del inspector
  while (true) {
    // Comprueba si hay datos enviados desde la terminal Bluetooth
    if (SerialBT.available()) {
      String cmd = SerialBT.readStringUntil('\n'); // Lee el comando hasta el salto de línea
      cmd.trim(); // Limpia espacios en blanco

      // Comprueba si el comando es "SALIR" para terminar la función
      if (cmd.equalsIgnoreCase("SALIR")) {
        SerialBT.println("🔙 Volviendo al menú de pruebas...");
        break; // Sale del bucle while
      }

      SerialBT.println("> Enviando: " + cmd);
      modem.sendAT(cmd); // Envía el comando al módem usando la librería TinyGsm

      String res = ""; // String para almacenar la respuesta del módem
      // Espera una respuesta por hasta 2000 ms. Si la recibe (código 1), la imprime.
      if (modem.waitResponse(2000L, res) == 1) {
        SerialBT.println("< Respuesta del Módem:");
        SerialBT.print(res);
      } else {
        SerialBT.println("< No se recibió respuesta o hubo un timeout.");
      }
      SerialBT.println("----------------------------------------");
    }
  }
}
```

## mostrarConfiguracionActual()
Imprime un resumen completo y bien formateado de todos los parámetros de configuración actuales del sistema en la terminal Bluetooth, permitiendo al usuario verificar el estado del equipo de un vistazo.

```c
void mostrarConfiguracionActual() {
  // Obtiene la hora actual del RTC
  DateTime now = rtc.now();
  char buf[32]; // Buffer para almacenar la fecha/hora formateada
  // Formatea la fecha y hora en un string legible
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  // Imprime la información general del equipo
  SerialBT.println("\n📄 ===== CONFIGURACIÓN ACTUAL DEL EQUIPO =====");
  SerialBT.println("Firmware: " + String(FIRMWARE_VERSION));
  SerialBT.println("Equipo: " + String(config.nombre_equipo));
  SerialBT.println("Num. Serie: " + String(config.numSerie));
  SerialBT.println("Intervalo de medición: " + String(config.intervalo_minutos) + " min");
  SerialBT.println("Hora actual del sistema: " + String(buf));

  // Imprime la configuración de comunicación SMS
  SerialBT.println("\n--- Envio de datos y notificaciones SMS ---");
  SerialBT.println("Número DATOS: " + String(config.numero_SMSDatos));
  SerialBT.println("Número NOTIFICACIONES: " + String(config.numero_SMSNotif));
  SerialBT.println("Reintentos: " + String(config.reintentos_envio_sms));

  // Imprime el estado de los módulos opcionales
  SerialBT.println("\n--- Módulos Adicionales ---");
  SerialBT.println("Modbus (Piranómetro): " + String(config.usar_modbus ? "Activado" : "Desactivado"));
  SerialBT.println("LEDs de estado: " + String(config.usar_leds_estado ? "Activados" : "Desactivados"));

  // Imprime los parámetros de calibración de los sensores
  SerialBT.println("\n--- Calibración de Sensores ---");
  SerialBT.println("Tolerancia ADC: " + String(config.tolerancia_adc));
  SerialBT.println("Suelo S30 (Mojado/Seco): " + String(config.sueloS30_min) + " / " + String(config.sueloS30_max));
  SerialBT.println("Suelo S15 (Mojado/Seco): " + String(config.sueloS15_min) + " / " + String(config.sueloS15_max));

  char addr_buf[24]; // Buffer para almacenar la dirección formateada del sensor
  SerialBT.println("\n--- Direcciones de Sensores de Temperatura ---");
  // Formatea la dirección de 8 bytes del sensor 1 en un string hexadecimal
  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo1_addr[0], config.tempSuelo1_addr[1], config.tempSuelo1_addr[2], config.tempSuelo1_addr[3],
           config.tempSuelo1_addr[4], config.tempSuelo1_addr[5], config.tempSuelo1_addr[6], config.tempSuelo1_addr[7]);
  SerialBT.println("Sensor 1: " + String(addr_buf));
  // Formatea la dirección de 8 bytes del sensor 2
  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo2_addr[0], config.tempSuelo2_addr[1], config.tempSuelo2_addr[2], config.tempSuelo2_addr[3],
           config.tempSuelo2_addr[4], config.tempSuelo2_addr[5], config.tempSuelo2_addr[6], config.tempSuelo2_addr[7]);
  SerialBT.println("Sensor 2: " + String(addr_buf));

  SerialBT.println("✅ Fin de configuración.");
}
```
## menuPruebas()
Muestra un submenú dedicado a las herramientas de diagnóstico, permitiendo al usuario verificar cada componente de hardware de forma aislada para facilitar la detección de fallos.

```c
void menuPruebas() {
  // Bucle que mantiene el menú activo hasta que el usuario elija la opción de salir
  while (true) {
    // Imprime las opciones del menú de pruebas en la terminal Bluetooth
    SerialBT.println("\n📶 Menú de pruebas:");
    SerialBT.println("1️⃣ Enviar datos de prueba mediante SMS 🌐");
    SerialBT.println("2️⃣ Enviar notificación mediante SMS 🔔");
    SerialBT.println("3️⃣ Probar sensores de temperatura de suelo 🌡");
    SerialBT.println("4️⃣ Probar sensores de humedad 🌱");
    SerialBT.println("5️⃣ Escanear bus OneWire 🔍");
    SerialBT.println("6️⃣ Inspector de Comandos AT 🕵️");
    SerialBT.println("0️⃣ Volver al menú principal 🔙");
    SerialBT.print("🔸 Elige una opción: ");

    // Lee la opción seleccionada por el usuario
    String opcion = leerLineaBluetooth();
    opcion.toUpperCase(); // Convierte la entrada a mayúsculas para flexibilidad

    // Llama a la función de diagnóstico correspondiente a la opción
    if (opcion == "1") probarEnvioDatos();
    else if (opcion == "2") probarEnvioNotif();
    else if (opcion == "3") probarSensoresTemperatura();
    else if (opcion == "4") probarSensoresHumedad();
    else if (opcion == "5") escanearDispositivosOneWire();
    else if (opcion == "6") inspectorComandosAT();
    else if (opcion == "0") break; // Si la opción es "0", rompe el bucle y sale del menú
    else SerialBT.println("❌ Opción no válida."); // Informa al usuario de una entrada incorrecta
  }
}
```

## probarEnvioNotif()
Función de diagnóstico que envía un mensaje SMS de prueba al número de teléfono configurado para notificaciones. Verifica que el módem esté activo y lo inicializa si es necesario.

```c
void probarEnvioNotif(){
  // Primero, comprueba si el uso del SIM800 está habilitado en la configuración
  if (!config.usar_sim800) {
    SerialBT.println("⚠️ El SIM800L está desactivado. Actívalo desde el menú.");
    return; // Sale de la función si está desactivado
  }
  // Si está habilitado, comprueba si ya fue inicializado
  if (!sim800Inicializado) {
    SerialBT.println("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L(); // Intenta inicializarlo
  }
  // Si el intento de inicialización falló, aborta la prueba
  if (!sim800Inicializado) {
    SerialBT.println("❌ No se pudo inicializar el módulo SIM. Abortando prueba.");
    return;
  }

  // Crea el contenido del mensaje de prueba
  String mensajePruebaNotif = "Mensaje de prueba desde " + String(config.nombre_equipo);

  // Informa al usuario de la acción que se va a realizar
  SerialBT.println("📲 Enviando SMS de notificacion al número " + String(config.numero_SMSNotif));
  SerialBT.println("   Contenido: " + mensajePruebaNotif);

  // Llama a la función de bajo nivel para realizar el envío
  enviarSMS(config.numero_SMSNotif, mensajePruebaNotif);
}
```
## enviarSMS()
Función de bajo nivel que se encarga de enviar un SMS. Incluye un mecanismo de reintentos en caso de fallo, que incluye un reinicio físico del módem para intentar recuperarlo de un posible bloqueo.

```c
bool enviarSMS(const char* numero, const String& mensaje) {
  // Doble verificación: no intentar enviar si el módulo está deshabilitado
  if (!config.usar_sim800) {
    debugPrint("📛 Envio de SMS desactivado por configuracion.");
    return false; // Devuelve 'falso' indicando que no se envió
  }
  // Si el módem no está listo, intenta iniciarlo
  if (!sim800Inicializado) {
    iniciarSIM800L();
  }

  // Bucle 'for' que intentará enviar el SMS hasta el número de veces configurado
  for (int intento = 1; intento <= config.reintentos_envio_sms; intento++) {
    debugPrint("📤 Enviando SMS a " + String(numero) + " (intento " + String(intento) + ")...");

    // Intenta enviar si hay red. Si tiene éxito, informa y devuelve 'true'.
    if (modem.isNetworkConnected() && modem.sendSMS(numero, mensaje)) {
      debugPrint("✅ SMS enviado correctamente.");
      return true;
    } else {
      // Si el envío falla, inicia un procedimiento de recuperación
      debugPrint("⚠️ Error al enviar SMS. Reiniciando modem...");
      digitalWrite(MODEM_PWR, LOW);     // Apaga el módem (corte de energía)
      delay(2000);
      digitalWrite(MODEM_PWR, HIGH);    // Vuelve a encender el módem
      delay(2000);
      sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX); // Reinicia la comunicación serial
      modem.restart();                  // Envía un comando de reinicio por software
      delay(3000);
    }

    // Espera un tiempo corto antes del siguiente intento
    delay(config.espera_entre_reintentos_sms_ms);
  }

  // Si el bucle termina sin haber logrado enviar el SMS, informa del fallo
  debugPrint("❌ No se pudo enviar el SMS despues de varios intentos.");
  return false; // Devuelve 'falso'
}
```

## probarEnvioDatos()
Función de diagnóstico que construye una cadena de datos CSV simulados y la envía por SMS al número configurado para datos. Es similar a probarEnvioNotif pero usa datos de ejemplo.

```c
void probarEnvioDatos(){
  // Comprueba si el módulo SIM800 está habilitado en la configuración
  if (!config.usar_sim800) {
    SerialBT.println("⚠️ El SIM800L está desactivado. Actívalo desde el menú.");
    return;
  }
  // Si el módem no está listo, intenta iniciarlo
  if (!sim800Inicializado) {
    SerialBT.println("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L();
  }
  // Si la inicialización falla, aborta la prueba
  if (!sim800Inicializado) {
    SerialBT.println("❌ No se pudo inicializar el módulo SIM. Abortando prueba.");
    return;
  }

  // Construye una cadena de texto CSV con datos de ejemplo fijos
  String mensajeDePruebaCSV = String(config.nombre_equipo) + "," +
                              fechaActual(rtc.now()) + "," +
                              "25.5," + // tempAmbiente
                              "60.1," + // humAmbiente
                              "55," +   // sueloS30
                              "22.3," + // tempSuelo1
                              "15000," +// s30_valor
                              "65," +   // sueloS15
                              "23.1," + // tempSuelo2
                              "14000," +// s15_valor
                              "850";    // radiacion

  // Informa al usuario sobre la acción y el contenido del mensaje
  SerialBT.println("📲 Enviando SMS de prueba al número " + String(config.numero_SMSDatos));
  SerialBT.println("   Contenido: " + mensajeDePruebaCSV);
  // Llama a la función de bajo nivel para realizar el envío
  enviarSMS(config.numero_SMSDatos, mensajeDePruebaCSV);
}
```

## getTemp()
Función auxiliar que lee la temperatura en grados Celsius de un sensor DS18B20 específico a través de su dirección única. También maneja el caso en que el sensor esté desconectado.

```c
float getTemp(DeviceAddress deviceAddress) {
  // Llama a la librería DallasTemperature para obtener la temperatura del sensor especificado
  float tempC = sensors.getTempC(deviceAddress);

  // La librería devuelve un valor especial (-127) si no puede comunicarse con el sensor
  if (tempC == DEVICE_DISCONNECTED_C) {
    return -127.0; // Retorna el código de error
  }

  // Si la lectura fue exitosa, retorna el valor de temperatura
  return tempC;
}
```

## probarSensoresHumedad()
Función de diagnóstico que lee los sensores de humedad del suelo en tiempo real, mostrando tanto el valor crudo del ADC como el porcentaje de humedad calculado, lo que permite verificar su funcionamiento y calibración.

```c
void probarSensoresHumedad() {
  SerialBT.println("\n🌱 Probando sensores de humedad del suelo...");

  // Verifica si el ADC externo (ADS1115) está funcionando
  if (!ads.begin()) {
    SerialBT.println("❌ Error: No se pudo inicializar el ADS1115.");
    return;
  }

  SerialBT.println("Lecturas en tiempo real (crudo / %):");

  // --- Lectura del primer sensor (S30) ---
  int rawS30 = leerPromedioADC(0, 5, 50, false); // Lee el valor crudo del canal 0 del ADC
  int humS30 = calcularPorcentajeHumedad(rawS30, config.sueloS30_max, config.sueloS30_min); // Lo convierte a %
  SerialBT.print("🔹 Sensor S30: " + String(rawS30) + " / ");
  if (humS30 == -1) { // Lógica para interpretar los valores de saturación
    SerialBT.println("SAT-H (Saturado Húmedo)");
  } else if (humS30 == 101) {
    SerialBT.println("SAT-S (Saturado Seco)");
  } else {
    SerialBT.println(String(humS30) + "%");
  }

  // --- Lectura del segundo sensor (S15) ---
  int rawS15 = leerPromedioADC(1, 5, 50, false); // Lee el valor crudo del canal 1 del ADC
  int humS15 = calcularPorcentajeHumedad(rawS15, config.sueloS15_max, config.sueloS15_min); // Lo convierte a %
  SerialBT.print("🔹 Sensor S15: " + String(rawS15) + " / ");
    if (humS15 == -1) {
    SerialBT.println("SAT-H (Saturado Húmedo)");
  } else if (humS15 == 101) {
    SerialBT.println("SAT-S (Saturado Seco)");
  } else {
    SerialBT.println(String(humS15) + "%");
  }
}
```

## probarSensoresTemperatura()
Función de diagnóstico que lee y muestra en la terminal las temperaturas actuales de los dos sensores de suelo DS18B20, permitiendo verificar que ambos estén conectados y funcionando correctamente.

```c
void probarSensoresTemperatura() {
  SerialBT.println("🌡️  Probando sensores de temperatura DS18B20...");
  sensors.begin(); // Inicializa la librería
  SerialBT.print("Solicitando lecturas...");
  // Envía un comando a TODOS los sensores en el bus para que midan la temperatura
  sensors.requestTemperatures();
  SerialBT.println(" OK"); // La medición toma un tiempo, pero el código no se bloquea aquí

  // --- Leer y mostrar temperatura del sensor 1 ---
  // Llama a la función auxiliar para obtener la temperatura usando la dirección guardada
  float tempS1 = getTemp(config.tempSuelo1_addr);
  SerialBT.print("🔹 Sensor de Suelo 1: ");
  // Comprueba si la función devolvió el código de error de desconexión
  if (tempS1 == DEVICE_DISCONNECTED_C) {
    SerialBT.println("❌ Error, sensor no encontrado.");
  } else {
    // Si la lectura es válida, la imprime
    SerialBT.print(tempS1);
    SerialBT.println(" °C");
  }

  // --- Leer y mostrar temperatura del sensor 2 ---
  // Repite el mismo proceso para el segundo sensor
  float tempS2 = getTemp(config.tempSuelo2_addr);
  SerialBT.print("🔹 Sensor de Suelo 2: ");
  if (tempS2 == DEVICE_DISCONNECTED_C) {
    SerialBT.println("❌ Error, sensor no encontrado.");
  } else {
    SerialBT.print(tempS2);
    SerialBT.println(" °C");
  }
}
```

## iniciarServidorWeb()
Configura el ESP32 como un Punto de Acceso WiFi y levanta un servidor web. Este servidor tiene diferentes "endpoints" (rutas) para listar/descargar/eliminar archivos de la SD y para recibir un nuevo firmware para la actualización OTA (Over-the-Air).

```c
void iniciarServidorWeb() {
  // Configura el modo WiFi como Punto de Acceso (AP)
  WiFi.mode(WIFI_AP);
  // Inicia el AP con el nombre (SSID) y contraseña definidos
  WiFi.softAP(ssid_ap, password_ap);

  IPAddress ip = WiFi.softAPIP(); // Obtiene la dirección IP del servidor
  debugPrint("📡 Punto de acceso iniciado en IP: " + ip.toString());

  // --- Definición de las rutas del servidor web ---

  // Ruta raíz ("/"): Muestra la página principal
  servidorWeb.on("/", HTTP_GET, []() {
    File root = SD.open("/"); // Abre el directorio raíz de la SD
    // Comienza a construir la página HTML como un string
    String html = R"rawliteral(
      <!DOCTYPE html><html>...</html>
    )rawliteral";

    // Bucle para leer todos los archivos de la SD
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break; // Si no hay más archivos, sale del bucle
      // Si el archivo es un .csv, lo agrega como un elemento a la lista en el HTML
      if (String(entry.name()).endsWith(".csv")) {
        html += "...<li>...</li>...";
      }
      entry.close();
    }
    servidorWeb.send(200, "text/html", html); // Envía la página HTML completa al cliente
  });

  // Ruta "/log": Maneja la descarga de archivos
  servidorWeb.on("/log", HTTP_GET, []() {
    String path = "/" + servidorWeb.arg("f"); // Obtiene el nombre del archivo de la URL
    if (!SD.exists(path)) { /* ... manejo de error ... */ }
    File f = SD.open(path, FILE_READ);
    // Envía el archivo al navegador con cabeceras que fuerzan la descarga
    servidorWeb.streamFile(f, "text/csv");
    f.close();
  });

  // Ruta "/delete": Maneja la eliminación de archivos
  servidorWeb.on("/delete", HTTP_GET, []() {
    String path = "/" + servidorWeb.arg("f");
    if (SD.remove(path)) { /* ... envía mensaje de éxito ... */ }
    else { /* ... envía mensaje de error ... */ }
  });

  // Ruta "/update" (GET): Muestra la página para subir el nuevo firmware
  servidorWeb.on("/update", HTTP_GET, []() {
    // Envía un string HTML que contiene el formulario de subida de archivos
    servidorWeb.send(200, "text/html", R"rawliteral(...)rawliteral");
  });

  // Ruta "/update" (POST): Procesa el archivo de firmware subido
  servidorWeb.on("/update", HTTP_POST,
    []() { // Esta parte se ejecuta CUANDO la subida TERMINA
      // Envía un mensaje de éxito/fallo y reinicia el ESP32 si todo fue bien
      servidorWeb.send(200, "text/plain", ...);
      if (!Update.hasError()) ESP.restart();
    },
    []() { // Esta parte se ejecuta MIENTRAS el archivo se está subiendo
      HTTPUpload& upload = servidorWeb.upload();
      // Gestiona los fragmentos del archivo y los escribe en la memoria flash
      if (upload.status == UPLOAD_FILE_START) { Update.begin(...); }
      else if (upload.status == UPLOAD_FILE_WRITE) { Update.write(...); }
      else if (upload.status == UPLOAD_FILE_END) { Update.end(true); }
    }
  );

  servidorWeb.begin(); // Inicia el servidor
  debugPrint("🌐 Servidor iniciado. Conéctate a 'HydroSense-AP'");
}
```

## obtenerNombreLogSemanal()
Función de utilidad que genera el nombre de archivo para el log (ej: log_2025-09-08.csv) basado en la fecha del lunes de la semana actual. Esto permite organizar los datos en archivos semanales.

```c
String obtenerNombreLogSemanal(const DateTime& dt) {
  // Obtiene el día de la semana (Lunes=1, ..., Domingo=7 o 0)
  int diaSemana = dt.dayOfTheWeek();
  // Calcula cuántos días hay que restar a la fecha actual para llegar al lunes
  int diasARestar = (diaSemana == 0) ? 6 : diaSemana - 1;

  // Crea un nuevo objeto DateTime que representa el inicio de la semana (lunes)
  DateTime inicioSemana = dt - TimeSpan(diasARestar, 0, 0, 0);

  char nombre[32]; // Buffer para el nombre del archivo
  // Formatea el nombre del archivo usando la fecha de inicio de la semana
  snprintf(nombre, sizeof(nombre), "log_%04d-%02d-%02d.csv",
           inicioSemana.year(), inicioSemana.month(), inicioSemana.day());
  return String(nombre); // Devuelve el nombre generado
}
```
## guardarEnSD()
Guarda una línea de datos en formato CSV en el archivo de log semanal correspondiente en la tarjeta SD. Si el archivo no existe o está vacío, primero escribe una fila de encabezado.

```c
void guardarEnSD(const String& lineaCSV) {
  // Obtiene la fecha y hora actual para determinar el nombre del archivo
  const DateTime now = rtc.now();
  // Llama a la función auxiliar para obtener el nombre del log de la semana actual
  const String nombreLog = obtenerNombreLogSemanal(now);
  // Construye la ruta completa del archivo
  const String path = "/" + nombreLog;
  // Comprueba si el archivo existe ANTES de abrirlo.
  const bool nuevoArchivo = !SD.exists(path);

  // Abre el archivo en modo "APPEND" (añadir al final)
  logFile = SD.open(path, FILE_APPEND);
  // Si no se pudo abrir el archivo, es un error crítico
  if (!logFile) {
    debugPrint("❌ Error al abrir archivo " + nombreLog);
    actualizarEstado(FALLO_CRITICO); // Llama a la gestión de errores críticos
    return;
  }

  // Escribe la cabecera CSV solo si el archivo es nuevo o está vacío
  if (nuevoArchivo || logFile.size() == 0) {
    const String encabezado = "Equipo,Fecha y hora,Temp. ambiente [C],...";
    logFile.println(encabezado);
  }

  // Escribe la línea de datos CSV en el archivo
  if (!logFile.println(lineaCSV)) {
    // Si la escritura falla, es un error crítico
    debugPrint("❌ Error al escribir en " + nombreLog);
    actualizarEstado(FALLO_CRITICO);
  } else {
    // Si la escritura es exitosa, lo informa
    debugPrint("📥 Datos guardados en " + nombreLog);
  }

  // Cierra el archivo para asegurar que los datos se guarden físicamente en la tarjeta
  logFile.close();
}
```
## menuBluetoothGeneral()
Es la función principal del modo de configuración. Muestra el menú de opciones al usuario y gestiona la navegación a las diferentes sub-rutinas de configuración, prueba y mantenimiento.

```c
void menuBluetoothGeneral() {
  // Bucle que espera hasta que un dispositivo se conecte vía Bluetooth
  while (!SerialBT.hasClient()) {
    debugPrint("⏳ Esperando cliente Bluetooth...");
    delay(500);
  }

  // Bucle principal que mantiene el menú activo
  while (true) {
    // Imprime el menú de opciones en la terminal Bluetooth
    SerialBT.println();
    SerialBT.println("📡 === MODO CONFIGURACIÓN BLUETOOTH ===");
    SerialBT.println("0️⃣ Configuración rápida de despliegue 🚀");
    // ... (más opciones)
    SerialBT.println("8️⃣ Salir del modo Bluetooth 🚪");
    SerialBT.println("🔸 Selecciona una opción: ");

    // Lee la opción numérica ingresada por el usuario
    const int8_t opcion = leerEnteroBluetooth();

    // Estructura 'switch' para ejecutar la acción correspondiente a la opción
    switch (opcion) {
      case 0: configuracionRapidaDespliegue(); break;
      case 1: mostrarConfiguracionActual(); break;
      case 2: editarConfiguracion(); break;
      case 3: menuCalibracionSensores(); break;
      case 4: guardarConfigEnArchivo(); break;
      case 5: sincronizarHoraPorBluetooth(); break;
      case 6: // Opción para reiniciar el dispositivo
        SerialBT.println("♻️ Reiniciando...");
        delay(1000);
        ESP.restart(); // Reinicia el microcontrolador
        break;
      case 7: menuPruebas(); break;
      case 8: // Opción para salir del modo de configuración
        SerialBT.println("🚪 Saliendo del modo Bluetooth...");
        return; // 'return' sale de la función, terminando el modo Bluetooth
      default: // Se ejecuta si la opción no es válida
        SerialBT.println("❌ Opción inválida. Intenta de nuevo.");
    }
  }
}
```
## configuracionRapidaDespliegue()
Implementa un asistente de configuración guiado (wizard) que pregunta al usuario sobre los parámetros más importantes (nombre, hora, calibración) para una puesta en marcha rápida del equipo en campo.

```c
void configuracionRapidaDespliegue() {
  // Bandera para determinar si es necesario guardar los cambios al final
  bool configModificada = false;
  SerialBT.println("\n🚀 Iniciando configuración rápida de despliegue...\n");

  // Muestra la configuración actual para que el usuario tenga contexto
  mostrarConfiguracionActual();

  // Pregunta si se desea cambiar el nombre del equipo
  SerialBT.println("\n¿Deseas cambiar el nombre del equipo? (s/n): ");
  String respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();
  if (respuesta == "s" || respuesta == "si") {
    editarCampoConfig("nombre del equipo", config.nombre_equipo, sizeof(config.nombre_equipo));
    configModificada = true; // Marca que se hizo un cambio
  }

  // Pregunta si se desea actualizar la hora
  SerialBT.println("\n¿Deseas actualizar la hora del sistema? (s/n): ");
  respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();
  if (respuesta == "s" || respuesta == "si") {
    sincronizarHoraPorBluetooth();
    configModificada = true;
  }

  // Pregunta si se desea iniciar la calibración de sensores
  SerialBT.println("\n¿Deseas iniciar el proceso de calibración de sensores de suelo? (s/n): ");
  respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();
  if (respuesta == "s" || respuesta == "si") {
    calibrarSensoresSuelo();
    // La calibración se guarda en RAM, se debe guardar explícitamente después.
  }

  // Si se modificó algún parámetro, se guardan los cambios en la SD
  if (configModificada) {
    SerialBT.println("💾 Guardando configuración...");
    if (guardarConfigEnArchivo()) {
      SerialBT.println("✅ Configuración guardada exitosamente.");
    } else {
      SerialBT.println("❌ Error al guardar la configuración.");
    }
  }

  // Pregunta al usuario si desea reiniciar el equipo para aplicar los cambios y comenzar la operación
  SerialBT.println("\n✅ Configuración rápida completada.");
  SerialBT.println("Ya puedes instalar el equipo 🚀");
  SerialBT.println("¿Deseas reiniciar el sistema ahora? (s/n): ");
  respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();

  if (respuesta == "s" || respuesta == "si") {
    SerialBT.println("🔄 Reiniciando sistema...");
    delay(1000);
    ESP.restart();
  } else {
    SerialBT.println("⏹️ Reinicio cancelado. Puedes continuar configurando el sistema.");
  }
}
```
## parpadearAmbosLeds()
Función de utilidad que hace parpadear los LEDs verde y rojo simultáneamente un número determinado de veces. Se usa para dar una señal visual clara y distintiva al usuario.

```c
void parpadearAmbosLeds(uint8_t cantidad) {
  // Bucle que se repite el número de veces especificado en 'cantidad'
  for (uint8_t i = 0; i < cantidad; i++) {
    digitalWrite(LED_VERDE, HIGH);  // Enciende el LED verde
    digitalWrite(LED_ROJO, HIGH);   // Enciende el LED rojo
    delay(500);                     // Espera medio segundo con los LEDs encendidos
    digitalWrite(LED_VERDE, LOW);   // Apaga el LED verde
    digitalWrite(LED_ROJO, LOW);    // Apaga el LED rojo
    delay(200);                     // Espera un corto tiempo antes del siguiente parpadeo
  }
}
```
## detectarModoInicio()
Función crítica ejecutada al arranque. Determina el modo de operación del sistema midiendo cuánto tiempo se mantiene presionado un botón físico. Esto permite al usuario elegir entre el modo normal, el modo de configuración por Bluetooth o el modo de servidor web.

```c
void detectarModoInicio() {
  // Configura el pin del botón como entrada con una resistencia pull-up interna.
  // Esto significa que el pin leerá HIGH por defecto y LOW cuando se presione.
  pinMode(PIN_BLUETOOTH_ACTIVADOR, INPUT_PULLUP);

  // Comprueba si el botón está presionado al momento de arrancar
  if (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
    unsigned long t0 = millis(); // Guarda el tiempo de inicio de la pulsación
    // Banderas para dar feedback visual solo una vez por umbral
    bool mostrado_5s = false;
    bool mostrado_10s = false;

    // Bucle que se ejecuta mientras el botón se mantenga presionado
    while (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
      unsigned long ahora = millis();
      unsigned long duracion = ahora - t0; // Calcula el tiempo transcurrido

      // Si han pasado 5 segundos y no se ha mostrado el feedback...
      if (duracion >= 5000 && !mostrado_5s) {
        parpadearAmbosLeds(3); // ...parpadea los LEDs para indicar modo Bluetooth
        mostrado_5s = true;   // ...y activa la bandera para no repetir.
      }
      // Lo mismo para el umbral de 10 segundos
      if (duracion >= 10000 && !mostrado_10s) {
        parpadearAmbosLeds(3); // Indica modo Servidor Web
        mostrado_10s = true;
      }
      delay(50); // Pequeña pausa para no sobrecargar el procesador
    }

    // Una vez que el botón se suelta, se calcula la duración final
    unsigned long duracion_final = millis() - t0;

    // Decide el modo de operación basado en la duración final
    if (duracion_final >= 10000) { // Más de 10 segundos
      modoBluetooth = false;
      modoServidorWeb = true;
    } else if (duracion_final >= 5000) { // Entre 5 y 10 segundos
      modoBluetooth = true;
      modoServidorWeb = false;
    }
    // Si fue menos de 5 segundos, no se hace nada y el sistema arranca en modo normal.
  }
}
```
## sincronizarHoraPorBluetooth()
Permite al usuario ajustar manualmente la fecha y hora del RTC enviando un string con un formato específico (YYYY-MM-DD HH:MM:SS) a través de la terminal Bluetooth.

```c
bool sincronizarHoraPorBluetooth() {
  // Imprime las instrucciones para el usuario en la terminal Bluetooth
  debugPrint("⏳ Envia la hora por Bluetooth en formato:");
  debugPrint("📤 YYYY-MM-DD HH:MM:SS");
  debugPrint("➡️ Ejemplo: 2025-07-06 18:45:00");

  String entrada = "";

  // Bucle que espera hasta recibir un string con la longitud correcta
  while (entrada.length() < 19) {
    if (SerialBT.available()) {
      entrada = SerialBT.readStringUntil('\n'); // Lee la línea enviada por el usuario
      entrada.trim();
      debugPrint("📨 Recibido: " + entrada);
    }
    delay(100);  // Pequeña pausa para reducir el uso de CPU
  }

  // --- Procesamiento del string recibido ---
  // Usa el método substring() para extraer cada parte de la fecha/hora
  // y toInt() para convertirla a un número entero.
  int year   = entrada.substring(0,  4).toInt();
  int month  = entrada.substring(5,  7).toInt();
  int day    = entrada.substring(8, 10).toInt();
  int hour   = entrada.substring(11,13).toInt();
  int minute = entrada.substring(14,16).toInt();
  int second = entrada.substring(17,19).toInt();

  // Valida que los valores parseados estén dentro de un rango lógico
  if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 59) {
    debugPrint("❌ Fecha u hora fuera de rango.");
    return false; // Devuelve 'false' si la validación falla
  }

  // Si los datos son válidos, ajusta el RTC con un nuevo objeto DateTime
  rtc.adjust(DateTime(year, month, day, hour, minute, second));

  // Imprime una confirmación con la nueva hora establecida
  char buf[30];
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
           year, month, day, hour, minute, second);
  debugPrint("✅ RTC ajustado manualmente:");
  debugPrint(String(buf));

  return true; // Devuelve 'true' si la sincronización fue exitosa
}
```
## iniciarSIM800L()
Gestiona la inicialización del módem SIM800L. Realiza un encendido por hardware (vía MOSFET), lo reinicia por software (comando AT) y espera la conexión a la red celular.

```c
bool iniciarSIM800L() {
  // Si la bandera indica que ya está inicializado, no hace nada más.
  if (sim800Inicializado) {
    debugPrint("✅ SIM800L ya estaba encendido");
    return true;
  } else {
    // Si no, procede con el encendido físico
    debugPrint("📶 Encendiendo SIM800L...");
    digitalWrite(MODEM_PWR, HIGH); // Activa el MOSFET que alimenta el módem
    delay(3000); // Espera a que el módem arranque
    parpadearLed(LED_VERDE, 3, true); // Feedback visual
    sim800Inicializado = true; // Actualiza la bandera
    debugPrint("✅ SIM800L encendido");
  }

  debugPrint("📡 Iniciando modem y buscando red...");
  // Inicia la comunicación serial en el puerto UART dedicado al módem
  sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  // Envía un comando de reinicio por software al módem
  if (!modem.restart()) {
    debugPrint("❌ Fallo al reiniciar el módem");
    actualizarEstado(FALLO_NO_CRITICO, "SIM800L", "Reinicio fallido");
    sim800Inicializado = false;
    return false;
  }
  delay(3000);

  // Espera a que el módem se registre en la red celular, con un tiempo de espera máximo
  if (modem.waitForNetwork(config.timeout_red)) {
    debugPrint("✅ Red celular encontrada");
    parpadearLed(LED_VERDE, 3, true);
    int csq = modem.getSignalQuality(); // Obtiene la calidad de la señal (0-31)
    String operador = modem.getOperator(); // Obtiene el nombre del operador
    debugPrint("📶 Señal: " + String(csq) + " (0-31)");
    debugPrint("🏢 Operador: " + operador);
    return true;
  }

  // Si no se conecta a la red dentro del tiempo límite
  debugPrint("❌ Sin red celular disponible");
  actualizarEstado(FALLO_NO_CRITICO, "SIM800L", "Sin red celular");
  sim800Inicializado = false;
  return false;
}
```
## iniciarRTC()
Inicializa la comunicación con el reloj en tiempo real DS3231 y verifica si la hora se ha perdido debido a un fallo de alimentación de la batería de respaldo.

```c
bool iniciarRTC() {
  debugPrint("⏰ Inicializando RTC DS3231...");

  // Intenta iniciar la comunicación con el RTC en el bus I2C
  if (!rtc.begin()) {
    debugPrint("❌ RTC no detectado");
    // Si no se detecta, es un fallo crítico para el sistema
    actualizarEstado(FALLO_CRITICO, "RTC", "No se detecto el RTC DS3231");
    return false;
  }

  // Comprueba una bandera interna del RTC que se activa si pierde la alimentación principal y de respaldo
  if (rtc.lostPower()) {
    debugPrint("⚠️ El RTC perdió la hora. Requiere ajuste manual.");
  } else {
    // Si la hora es válida, la lee y la imprime como confirmación
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    debugPrint("✅ RTC en funcionamiento. Hora actual: " + String(buf));
  }

  debugPrint("✅ RTC inicializado correctamente");
  return true;
}
```
## fechaActual()
Función de utilidad que convierte un objeto DateTime de la librería RTClib en un String con el formato estándar AAAA/MM/DD HH:MM:SS.

```c
String fechaActual(const DateTime& dt) {
  // Crea un array de caracteres (buffer) para almacenar el string formateado
  char buffer[32];

  // Usa snprintf para formatear la fecha y hora de forma segura dentro del buffer.
  // %04d: entero de 4 dígitos (con ceros a la izquierda, ej: 2025)
  // %02d: entero de 2 dígitos (con ceros a la izquierda, ej: 09)
  snprintf(buffer, sizeof(buffer), "%04d/%02d/%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());

  // Convierte el buffer de C a un objeto String de Arduino y lo devuelve
  return String(buffer);
}
```
## verificarModbus()
Realiza una lectura simple de un registro Modbus para verificar si la comunicación con el esclavo (el piranómetro) está funcionando correctamente. Devuelve true si la comunicación es exitosa.

```c
bool verificarModbus() {
  // Intenta leer 1 registro ('Holding Register') desde la dirección 0x0000 del esclavo Modbus.
  uint8_t result = node.readHoldingRegisters(0x0000, 1);

  // Compara el resultado de la transacción con el código de éxito de la librería.
  if (result == node.ku8MBSuccess) {
    return true; // La comunicación fue exitosa
  } else {
    // Si hubo un error, imprime el código para depuración.
    debugPrint("❌ Fallo Modbus. Codigo: " + String(result));
    return false; // La comunicación falló
  }
}
```
## leerRadiacionModbus()
Lee el valor de radiación del sensor Modbus. Incluye validación del rango de la lectura y devuelve códigos de error específicos si la comunicación falla (-1) o si el valor leído es anómalo (-2).

```c
int leerRadiacionModbus() {
  // Intenta leer el registro que contiene el valor de radiación.
  uint8_t result = node.readHoldingRegisters(0x0000, 1);

  // Si la transacción Modbus fue exitosa...
  if (result == node.ku8MBSuccess) {
    // ...recupera el valor del buffer de respuesta de la librería.
    int16_t valor = node.getResponseBuffer(0);

    // Realiza una validación de rango para asegurar que el dato sea coherente.
    if (valor >= 0 && valor <= 2000) {
      return valor; // Devuelve el valor de radiación si es válido.
    } else {
      debugPrint("⚠️ Valor de radiacion fuera de rango: " + String(valor));
      return -2; // Devuelve un código de error para valor fuera de rango.
    }
  } else {
    // Si la transacción Modbus falló...
    debugPrint("❌ Error en lectura Modbus. Codigo: " + String(result));
    return -1; // Devuelve un código de error para fallo de comunicación.
  }
}
```
## actualizarEstado()
Función central para la gestión de errores del sistema. Basado en el nivel de fallo (EstadoSistema), esta función actualiza los indicadores LED y puede tomar acciones drásticas como enviar una notificación por SMS o poner el dispositivo en modo de reposo indefinido en caso de un fallo crítico.

```c
void actualizarEstado(EstadoSistema estado, const String& id, const String& descripcion) {
  // Actualiza las variables globales que rastrean el estado del sistema
  estadoActual = estado;
  ultimoEstado = estado;

  // Selecciona la acción a tomar basada en el tipo de estado
  switch (estado) {
    // Si todo está OK, parpadea el LED verde
    case ESTADO_OK:
      digitalWrite(LED_ROJO, LOW);
      parpadearLed(LED_VERDE, 5, false);
      break;

    // Si el fallo es CRÍTICO
    case FALLO_CRITICO: {
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROJO, HIGH); // LED rojo fijo

      // Construye un mensaje de alerta detallado
      const String mensaje = "... FALLO CRITICO ...";
      // Envía la alerta por SMS al número de notificaciones
      enviarSMS(config.numero_SMSNotif, mensaje);

      // Apaga todos los periféricos para ahorrar energía
      digitalWrite(MODEM_PWR, LOW);
      digitalWrite(POWER_CTRL_PIN, LOW);
      delay(500);

      // Pone el ESP32 en deep sleep indefinido para prevenir más daños o gasto de batería
      esp_deep_sleep_start();
      break;
    }

    // Si el fallo es MEDIO
    case FALLO_MEDIO: {
      // Parpadea el LED rojo y envía una notificación SMS, pero permite que el sistema siga operando
      // ...
      break;
    }

    // Si el fallo NO es crítico
    case FALLO_NO_CRITICO:
      // Solo da una indicación visual (parpadeo rojo) sin enviar SMS
      digitalWrite(LED_VERDE, LOW);
      parpadearLed(LED_ROJO, 3, true);
      break;
  }
}
```
## parpadearLed()
Función de utilidad que hace parpadear un pin de LED específico. Permite controlar el número de parpadeos y su duración (corta o larga), haciéndola muy flexible para proporcionar diferentes tipos de feedback visual.

```c
void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo) {
  // Usa un operador ternario para definir los tiempos de encendido y apagado
  // Si 'largo' es true, el parpadeo es de 1s encendido y 0.5s apagado.
  const uint16_t onTime = largo ? 1000 : 300;
  // Si 'largo' es false, el parpadeo es de 0.3s encendido y 0.2s apagado.
  const uint16_t offTime = largo ? 500 : 200;

  // Bucle 'while' que se ejecuta el número de veces indicado en 'cantidad'.
  // La condición (cantidad--) es una forma compacta de iterar y decrementar.
  while (cantidad--) {
    digitalWrite(pinLed, HIGH); // Enciende el LED
    delay(onTime);              // Espera el tiempo de encendido
    digitalWrite(pinLed, LOW);  // Apaga el LED
    delay(offTime);             // Espera el tiempo de apagado
  }
}
```

## leerPromedioADC()
Toma múltiples muestras de un canal del ADC ADS1115 y devuelve el promedio. Esto es crucial para obtener una lectura analógica más estable y precisa, filtrando el ruido eléctrico.

```c
int leerPromedioADC(uint8_t canal, uint8_t muestras, uint16_t intervaloMs, bool debugBT) {
  long suma = 0; // Usa 'long' para evitar desbordamiento si se toman muchas muestras

  // Bucle 'for' que se repite el número de veces especificado en 'muestras'
  for (uint8_t i = 0; i < muestras; i++) {
    int valor = ads.readADC_SingleEnded(canal); // Lee el valor crudo del canal del ADC
    suma += valor; // Acumula la lectura en la variable 'suma'

    // Si la depuración por Bluetooth está activada y hay un cliente conectado...
    if (debugBT && bluetoothActivo && SerialBT.hasClient()) {
      // ...imprime cada muestra individual para facilitar la calibración.
      SerialBT.println("📡 Lectura " + String(i+1) + "/" +
                       String(muestras) + ": " + String(valor));
    }
    delay(intervaloMs); // Espera el tiempo definido entre cada muestra
  }
  // Devuelve el promedio de todas las lecturas tomadas
  return suma / muestras;
}
```

## calibrarSensoresSuelo()
Guía al usuario a través de un proceso interactivo en la terminal Bluetooth para calibrar los sensores de humedad del suelo, estableciendo los valores de referencia para "seco" y "mojado" que se usarán en los cálculos de porcentaje.

```c
void calibrarSensoresSuelo() {
  // Limpia cualquier dato residual en el buffer de entrada de Bluetooth
  while (SerialBT.available()) SerialBT.read();

  // Verifica que el módulo ADC esté funcionando antes de empezar
  if (!ads.begin()) {
    SerialBT.println("❌ ADS1115 no inicializado. Cancelando calibracion.");
    return;
  }

  SerialBT.println("🔧 Iniciando calibracion de sensores de suelo.");

  // === CALIBRACIÓN S30 ===
  // Pide al usuario que ponga el sensor en un medio seco y presione Enter
  SerialBT.println("➡️ Coloca el sensor S30 en ambiente SECO y presiona ENTER.");
  while (!SerialBT.available()) delay(100); // Espera la entrada del usuario
  while (SerialBT.available()) SerialBT.read(); // Limpia el buffer de nuevo
  delay(500);
  // Toma la lectura promedio para el estado "seco"
  int S30_seco = leerPromedioADC(0, 5, 1000, true);
  SerialBT.println("✅ Leido S30 seco: " + String(S30_seco));

  // Pide al usuario la lectura en un medio mojado
  SerialBT.println("➡️ Ahora coloca S30 en ambiente MOJADO y presiona ENTER.");
  while (!SerialBT.available()) delay(100);
  while (SerialBT.available()) SerialBT.read();
  delay(500);
  // Toma la lectura promedio para el estado "mojado"
  int S30_mojado = leerPromedioADC(0, 5, 1000, true);
  SerialBT.println("✅ Leido S30 mojado: " + String(S30_mojado));

  // === CALIBRACIÓN S15 (repite el mismo proceso para el segundo sensor) ===
  SerialBT.println("➡️ Coloca el sensor S15 en ambiente SECO y presiona ENTER.");
  // ... (código idéntico para el sensor S15) ...
  int S15_seco = leerPromedioADC(1, 5, 1000, true);
  SerialBT.println("✅ Leido S15 seco: " + String(S15_seco));

  SerialBT.println("➡️ Ahora coloca S15 en ambiente MOJADO y presiona ENTER.");
  // ... (código idéntico para el sensor S15) ...
  int S15_mojado = leerPromedioADC(1, 5, 1000, true);
  SerialBT.println("✅ Leido S15 mojado: " + String(S15_mojado));

  // === ACTUALIZACIÓN EN RAM ===
  // Guarda los valores de calibración en la estructura 'config' que está en la memoria RAM.
  // Nota: Los valores 'min' corresponden a mojado y 'max' a seco.
  config.sueloS30_min = S30_mojado;
  config.sueloS30_max = S30_seco;
  config.sueloS15_min = S15_mojado;
  config.sueloS15_max = S15_seco;

  SerialBT.println("✅ Calibración almacenada en memoria RAM.");
  SerialBT.println("💾 Recuerda guardar la configuración si deseas hacerla permanente.");
}
```

## debugPrint()
Función de utilidad centralizada para imprimir mensajes de depuración. Envía el mensaje tanto al monitor serie del PC (vía USB) como a la terminal Bluetooth si hay un cliente conectado, simplificando el monitoreo en diferentes escenarios.

```c
void debugPrint(const String& mensaje) {
  // Siempre imprime en el puerto Serie principal (conectado por USB)
  Serial.println(mensaje);

  // Comprueba si el modo Bluetooth está activo Y si hay un dispositivo conectado
  if (bluetoothActivo && SerialBT.hasClient()) {
    // Si ambas condiciones son ciertas, envía el mismo mensaje por Bluetooth
    SerialBT.println(mensaje);
  }
}
```

## iniciarSD()
Inicializa la comunicación con la tarjeta Micro SD. Es una función crítica; si la tarjeta no se detecta o falla al iniciar, el sistema no puede operar (ya que no puede leer la configuración ni guardar logs) y entra en un estado de fallo crítico.

```c
bool iniciarSD() {
  debugPrint("💾 Inicializando tarjeta microSD...");
  // Intenta iniciar la comunicación con la SD en el pin CS definido
  if (!SD.begin(SD_CS)) {
    // Si SD.begin() devuelve 'false', la inicialización falló
    debugPrint("❌ Fallo al inicializar microSD");
    // Llama a la función de estado para registrar un fallo CRÍTICO
    actualizarEstado(FALLO_CRITICO, "SD", "Fallo al inicializar la microSD");
    return false; // Devuelve 'false' para detener el proceso de arranque
  }
  // Si la inicialización fue exitosa
  debugPrint("✅ microSD inicializada");
  return true;
}
```

## iniciarADS1115()
Inicializa la comunicación I²C con el convertidor analógico-digital externo ADS1115. Este componente es crítico para leer los sensores de humedad, por lo que un fallo en su inicialización detiene el sistema.

```c
bool iniciarADS1115() {
  debugPrint("🔍 Inicializando ADS1115...");
  // Intenta iniciar la comunicación con el ADC en el bus I2C
  if (!ads.begin()) {
    // Si la librería no detecta el dispositivo, devuelve 'false'
    debugPrint("❌ Fallo al inicializar ADS1115");
    // Llama a la función de estado para registrar un fallo CRÍTICO
    actualizarEstado(FALLO_CRITICO, "ADS1115", "Fallo al inicializar el ADC");
    return false; // Detiene el proceso de arranque
  }
  debugPrint("✅ ADS1115 inicializado");
  return true;
}
```

## leerLineaBluetooth()
Función de utilidad que lee caracteres desde la terminal Bluetooth uno por uno hasta que recibe un salto de línea (\n) o un retorno de carro (\r), devolviendo el comando completo como un objeto String.


```c
String leerLineaBluetooth() {
  String entrada = ""; // String para acumular los caracteres leídos
  // Bucle infinito que espera activamente por la entrada del usuario
  while (true) {
    // Procesa los caracteres solo si hay datos disponibles en el buffer de Bluetooth
    while (SerialBT.available()) {
      char c = SerialBT.read(); // Lee un solo caracter
      // Comprueba si el caracter es un final de línea
      if (c == '\n' || c == '\r') {
        // Si ya se ha construido una cadena, la devuelve
        if (entrada.length() > 0) return entrada;
      } else {
        // Si no es un final de línea, lo añade a la cadena
        entrada += c;
      }
    }
    delay(10); // Pequeña pausa para que el bucle no consuma el 100% del CPU
  }
}
```

## leerEnteroBluetooth()
Función de ayuda que simplifica la lectura de números. Llama a leerLineaBluetooth() para obtener el texto y luego utiliza el método .toInt() de la clase String para convertirlo a un número entero.

```c
int leerEnteroBluetooth() {
  // Llama a la función para leer una línea de texto y, sobre el String devuelto,
  // invoca al método de conversión a entero.
  return leerLineaBluetooth().toInt();
}
```

## editarCampoConfig()
Función de ayuda genérica para editar un parámetro de configuración de tipo texto (cadena de caracteres). Muestra un mensaje al usuario, lee su entrada y la guarda en la variable de destino.

```c
void editarCampoConfig(const char* nombreCampo, char* destino, size_t maxLen) {
  // Imprime un mensaje en la terminal pidiendo al usuario el nuevo valor.
  // 'printf' permite formatear el string insertando el nombre del campo.
  SerialBT.printf("✏️ Ingresa nuevo valor para %s: ", nombreCampo);
  // Lee la línea de texto que el usuario envía por Bluetooth
  String entrada = leerLineaBluetooth();
  entrada.trim(); // Elimina espacios en blanco al inicio y al final
  // Copia el contenido del String a la variable de destino (array de char),
  // asegurando no exceder su tamaño máximo (maxLen).
  entrada.toCharArray(destino, maxLen);
  SerialBT.println("✅ Valor actualizado."); // Confirma la actualización
}
```

## editarCampoNumConfig()
Función de ayuda genérica para editar un parámetro de configuración de tipo numérico (entero). Funciona de manera similar a editarCampoConfig pero para números.

```c
void editarCampoNumConfig(const char* nombreCampo, int* destino) {
  // Pide al usuario el nuevo valor numérico
  SerialBT.printf("✏️ Ingresa nuevo valor numérico para %s: ", nombreCampo);
  // Llama a la función auxiliar que lee una línea y la convierte a entero
  int valor = leerEnteroBluetooth();
  // Asigna el nuevo valor a la variable de configuración original
  // usando un puntero ('*destino').
  *destino = valor;
  SerialBT.println("✅ Valor actualizado.");
}
```

## alternarCampoBool()
Función de ayuda para cambiar (alternar) el valor de un parámetro de configuración booleano (de true a false y viceversa), como activar o desactivar un módulo.

```c
void alternarCampoBool(const char* nombreCampo, bool* destino) {
  // Invierte el valor booleano actual. Si era 'true' se convierte en 'false' y viceversa.
  *destino = !(*destino);
  // Imprime el nuevo estado usando un operador ternario para mostrar un mensaje amigable
  SerialBT.printf("🔁 %s ahora está: %s\n", nombreCampo, *destino ? "✅ Activado" : "❌ Desactivado");
}
```

## menuCalibracionSensores()
Muestra un submenú dedicado a la calibración de los sensores de humedad del suelo, permitiendo al usuario iniciar el proceso interactivo o simplemente consultar los valores de calibración guardados actualmente.

```c
void menuCalibracionSensores() {
  // Bucle que mantiene el menú de calibración activo
  while (true) {
    // Imprime las opciones del menú
    SerialBT.println("\n🌱 === CALIBRACIÓN DE SENSORES DE HUMEDAD ===");
    SerialBT.println("1️⃣ Iniciar calibración interactiva");
    SerialBT.println("2️⃣ Mostrar valores actuales de calibración");
    SerialBT.println("3️⃣ Volver al menú anterior 🔙");
    SerialBT.print("🔸 Selecciona una opción: ");

    String opcion = leerLineaBluetooth(); // Lee la opción del usuario

    if (opcion == "1") {
      // Si elige '1', inicia el proceso de calibración guiado
      SerialBT.println("🧪 Iniciando proceso de calibración...");
      calibrarSensoresSuelo();
      SerialBT.println("✅ Calibración finalizada.");
    } else if (opcion == "2") {
      // Si elige '2', muestra los valores de calibración actuales guardados en la config
      SerialBT.println("📊 Valores actuales de calibración:");
      SerialBT.println("S30 (Mojado/Seco): " + String(config.sueloS30_min) + " / " + String(config.sueloS30_max));
      SerialBT.println("S15 (Mojado/Seco): " + String(config.sueloS15_min) + " / " + String(config.sueloS15_max));
    } else if (opcion == "3") {
      // Si elige '3', sale del menú
      SerialBT.println("🔙 Volviendo al menú anterior...");
      break; // Rompe el bucle
    } else {
      // Maneja entradas no válidas
      SerialBT.println("❌ Opción inválida.");
    }
  }
}
```

## guardarConfigEnArchivo()
Toma todos los parámetros de la estructura config que están en la memoria RAM, los serializa en un documento JSON y los escribe en el archivo config.json en la tarjeta SD para hacerlos permanentes.

```c
bool guardarConfigEnArchivo() {
  // Crea un objeto JsonDocument de la librería ArduinoJson para construir el JSON en memoria
  JsonDocument doc;

  // Rellena el documento JSON creando pares de clave-valor.
  // La clave es un string (ej: "nombre_equipo") y el valor se toma del struct 'config'.
  doc["nombre_equipo"] = config.nombre_equipo;
  doc["numSerie"] = config.numSerie;
  doc["sueloS30_min"] = config.sueloS30_min;
  // ... (se añaden todos los demás parámetros)

  // Las direcciones de los sensores son arrays de bytes, no pueden guardarse directamente en JSON.
  // Primero se formatean a un string hexadecimal legible (ej: "28:FF:...")
  char addr_buf[24];
  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo1_addr[0], config.tempSuelo1_addr[1], /*...*/);
  doc["tempSuelo1_addr"] = addr_buf; // Luego se guarda el string en el JSON

  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo2_addr[0], config.tempSuelo2_addr[1], /*...*/);
  doc["tempSuelo2_addr"] = addr_buf;

  // Abre el archivo config.json en modo escritura (sobrescribe el contenido anterior)
  File configFile = SD.open("/config.json", FILE_WRITE);
  if (!configFile) {
    debugPrint("❌ Error al abrir config.json para escritura.");
    return false;
  }
  // Escribe el documento JSON al archivo. Si la función devuelve 0, hubo un error.
  if (serializeJson(doc, configFile) == 0) {
    debugPrint("❌ Error al escribir en config.json.");
    configFile.close();
    return false;
  }

  // Cierra el archivo para asegurar que los cambios se guarden
  configFile.close();
  debugPrint("✅ Configuracion guardada.");
  return true;
}
```

## cargarConfiguracion()
Lee y parsea el archivo config.json de la tarjeta SD al arrancar el sistema. Si el archivo es válido, carga los valores en la estructura config. Si no existe o contiene errores, el sistema utilizará valores por defecto.

```c
bool cargarConfiguracion() {
  debugPrint("📂 Cargando configuracion desde config.json...");
  File configFile = SD.open("/config.json", FILE_READ);
  // Si el archivo no se puede abrir, se usan los valores por defecto
  if (!configFile) {
    debugPrint("❌ No se encontro config.json, se usarán valores por defecto.");
    return false;
  }

  JsonDocument doc; // Objeto para almacenar el JSON parseado
  // Lee el archivo y lo parsea. Si hay un error de sintaxis en el JSON, la función falla.
  if (deserializeJson(doc, configFile)) {
    debugPrint("❌ Error al parsear config.json");
    configFile.close();
    return false;
  }
  configFile.close();

  // --- Carga de Parámetros ---
  // Para cada parámetro, intenta leerlo del JSON. Si la clave no existe,
  // el operador '|' asigna un valor por defecto.
  strncpy(config.nombre_equipo, doc["nombre_equipo"] | "HydroSense", sizeof(config.nombre_equipo));
  config.intervalo_minutos = doc["intervalo_minutos"] | 15;
  // ... (se cargan todos los demás parámetros con sus respectivos valores por defecto)

  // Para las direcciones de los sensores, se lee el string del JSON...
  const char* addr1_str = doc["tempSuelo1_addr"] | "28:32:B0:87:00:2D:04:A4";
  // ...y luego se usa sscanf para parsearlo y guardarlo de vuelta en el array de bytes.
  sscanf(addr1_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &config.tempSuelo1_addr[0], /*...*/);

  const char* addr2_str = doc["tempSuelo2_addr"] | "28:DE:62:87:00:03:79:35";
  sscanf(addr2_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &config.tempSuelo2_addr[0], /*...*/);

  debugPrint("✅ Configuracion cargada.");
  return true;
}
```

## calcularPorcentajeHumedad()
Convierte el valor crudo del ADC de un sensor de humedad en un porcentaje (0-100%) utilizando una interpolación lineal entre los puntos de calibración 'seco' y 'mojado'. También maneja valores fuera de rango.

```c
int calcularPorcentajeHumedad(const int valor, const int seco, const int mojado) {
  // Caso especial para evitar una división por cero si la calibración no es válida
  if (seco == mojado) return 0;

  // Define un rango de tolerancia para los valores de saturación
  const int minimoPermitido = mojado - config.tolerancia_adc;
  const int maximoPermitido = seco + config.tolerancia_adc;
  // Si el valor es más bajo que el punto "mojado" (considerando la tolerancia),
  // se considera saturado de humedad.
  if (valor < minimoPermitido) return -1; // Código para SAT-H
  // Si el valor es más alto que el punto "seco", se considera saturado de sequedad.
  if (valor > maximoPermitido) return 101; // Código para SAT-S

  // Fórmula de interpolación lineal para mapear el rango [mojado, seco] al rango [100, 0]
  float porcentaje = 100.0 * (seco - valor) / (seco - mojado);

  // Redondea el resultado y lo restringe al rango 0-100 para evitar errores
  return constrain(round(porcentaje), 0, 100);
}
```
## iniciarModbus()
Configura el puerto serie, los pines de control del transceptor RS485 y el objeto de la librería ModbusMaster para la comunicación con el piranómetro. Finalmente, verifica que la comunicación sea exitosa.

```c
bool iniciarModbus() {
  debugPrint("🟡 Iniciando protocolo Modbus...");
  // Configura el pin de control de dirección del transceptor RS485
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW); // Lo pone en modo recepción por defecto

  // Inicia el puerto UART2 con los pines y la velocidad de baudios para Modbus
  Serial2.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);

  // Configura el objeto ModbusMaster para usar el esclavo ID 1 en el Serial2
  node.begin(1, Serial2);

  // --- Configuración de Callbacks para el control automático del pin DE/RE ---
  // Esta función se ejecutará automáticamente ANTES de transmitir
  node.preTransmission([]() {
    digitalWrite(RS485_DE_RE, HIGH); // Pone el transceptor en modo transmisión
  });
  // Esta función se ejecutará automáticamente DESPUÉS de transmitir
  node.postTransmission([]() {
    digitalWrite(RS485_DE_RE, LOW); // Vuelve a poner el transceptor en modo recepción
  });

  delay(200);
  parpadearLed(LED_VERDE, 3, true);
  debugPrint("✅ Modbus listo");

  // Realiza una prueba de comunicación para asegurarse de que el esclavo responde
  debugPrint("🛠️ Verificando conexion Modbus...");
  if (!verificarModbus()) {
    // Si no responde, es un fallo crítico
    actualizarEstado(FALLO_CRITICO, "MODBUS", "Fallo en la comunicacion RS485");
    return false;
  }

  parpadearLed(LED_VERDE, 3, true);
  debugPrint("📡 Modbus OK");
  return true;
}
```