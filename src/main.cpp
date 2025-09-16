/*
PIN LIST:
  + NOT TO USE:
    - D0
    - D1
    - D3

  + Temperatura suelo
    - DATA 13

  + DS3231 - 300 µA
    - SDA 15
    - SCL 4

  + SHT-10 - 1 mA
    - Data 16
    - Clock 17

  + Micro SD - 100mA
    - CS 5
    - SCK 18
    - MOSI 23
    - MISO 19
    - POWER 21

  + ADC sensores suelo
    - SDA 15
    - SCL 4
    - POWER 21

  + Pines RS485
    - RX 33
    - TX 32
    - DE/RE 27
    - POWER 21

  + SIM800L
    - RXD 26
    - TXD 25
    - POWER 22

  + LEDs de estado
    - Rojo 14
    - Verde 2

  + Bluetooth
    - Activador 12
*/

#define TINY_GSM_MODEM_SIM800
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <SHT1x-ESP.h>
#include <WebServer.h>
#include <Arduino.h>
#include <OneWire.h>
#include <Update.h>
#include <RTClib.h>
#include <Wire.h>
#include <WiFi.h>
#include <FS.h>
#include <SD.h>

// Temperatura suelo
constexpr int oneWireBus = 13;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// RTC DS3231
#define SDA_PIN 15
#define SCL_PIN 4
RTC_DS3231 rtc;

// Sensor SHT-10
#define DATA_PIN 16
#define CLOCK_PIN 17
SHT1x sht1x(DATA_PIN, CLOCK_PIN, SHT1x::Voltage::DC_3_3v);

// Micro SD
#define SD_CS 5
File logFile;

// ADC sensores suelo
Adafruit_ADS1115 ads;

// Pines RS485
#define RS485_DE_RE 27
#define RS485_RX 33
#define RS485_TX 32
ModbusMaster node;

// SIM800L
#define MODEM_RX 26
#define MODEM_TX 25
HardwareSerial sim800(1);
TinyGsm modem(sim800);
bool sim800Inicializado = false;

// MOSFET de control
#define POWER_CTRL_PIN 21
#define MODEM_PWR 22

// LEDs de estado
#define LED_VERDE 2
#define LED_ROJO 14

// Bluetooth
#define PIN_BLUETOOTH_ACTIVADOR 12
bool bluetoothActivo = false;
BluetoothSerial SerialBT;

// WebServer
auto ssid_ap = "HydroSense-AP";
auto password_ap = "hydrosense";
WebServer servidorWeb(80);

// Variables para control del sistema
#define FIRMWARE_VERSION "v2.7"
enum EstadoSistema {
  ESTADO_OK,
  FALLO_CRITICO,
  FALLO_MEDIO,
  FALLO_NO_CRITICO
};
EstadoSistema estadoActual;
RTC_DATA_ATTR uint8_t ciclos = 0;
RTC_DATA_ATTR EstadoSistema ultimoEstado = ESTADO_OK;
bool modoBluetooth = false;
bool modoServidorWeb = false;

// Configuracion del sistema
struct ConfiguracionSistema {
  int espera_entre_reintentos_sms_ms;
  uint8_t tempSuelo1_addr[8];
  uint8_t tempSuelo2_addr[8];
  int reintentos_envio_sms;
  char numero_SMSDatos[20];
  char numero_SMSNotif[20];
  char nombre_equipo[32];
  int intervalo_minutos;
  bool usar_leds_estado;
  uint16_t timeout_red;
  char archivo_log[32];
  int tolerancia_adc;
  char numSerie[16];
  int sueloS30_min;
  int sueloS30_max;
  int sueloS15_min;
  int sueloS15_max;
  bool usar_modbus;
  bool usar_sim800;
};
ConfiguracionSistema config;

// Prototipos
int leerPromedioADC(uint8_t canal, uint8_t muestras = 5,uint16_t intervaloMs = 50, bool debugBT = false);
void actualizarEstado(EstadoSistema estado, const String& id = "", const String& descripcion = "");
void editarCampoConfig(const char* nombreCampo, char* destino, size_t maxLen);
void editarDireccionSensor(const char* nombreSensor, uint8_t* destino);
void editarCampoNumConfig(const char* nombreCampo, int* destino);
void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo);
int calcularPorcentajeHumedad(int valor, int seco, int mojado);
void alternarCampoBool(const char* nombreCampo, bool* destino);
bool enviarSMS(const char* numero, const String& mensaje);
float getTemp(DeviceAddress deviceAddress);
void parpadearAmbosLeds(uint8_t cantidad);
void enviarDatos(const String& datosCSV);
void guardarEnSD(const String& lineaCSV);
String fechaActual(const DateTime& dt);
void debugPrint(const String& mensaje);
void configuracionRapidaDespliegue();
void escanearDispositivosOneWire();
bool sincronizarHoraPorBluetooth();
void mostrarConfiguracionActual();
void probarSensoresTemperatura();
String obtenerNombreLogSemanal();
void menuCalibracionSensores();
bool guardarConfigEnArchivo();
void probarSensoresHumedad();
void calibrarSensoresSuelo();
void menuBluetoothGeneral();
String leerLineaBluetooth();
void editarConfiguracion();
bool cargarConfiguracion();
void inspectorComandosAT();
int leerRadiacionModbus();
int leerEnteroBluetooth();
void iniciarServidorWeb();
void detectarModoInicio();
void probarEnvioNotif();
void probarEnvioDatos();
bool verificarModbus();
bool iniciarSIM800L();
bool iniciarADS1115();
bool iniciarModbus();
void menuPruebas();
bool iniciarRTC();
bool iniciarSD();

// Funciones
void setup() {
  delay(5000);

  pinMode(PIN_BLUETOOTH_ACTIVADOR, INPUT_PULLUP);
  pinMode(POWER_CTRL_PIN, OUTPUT);
  pinMode(MODEM_PWR, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_ROJO, LOW);

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Serial.begin(115200);
  delay(2000);

  digitalWrite(POWER_CTRL_PIN, HIGH);
  delay(500);
  if (!iniciarSD()) return;
  if (!cargarConfiguracion()) return;
  if (!iniciarADS1115()) return;
  if (!iniciarRTC()) return;

  detectarModoInicio();

  if (modoServidorWeb) {
    debugPrint("🌐 Iniciando modo servidor web para descarga...");
    iniciarServidorWeb();
    return;
  }

  if (modoBluetooth) {
    debugPrint("🔧 Iniciando modo Bluetooth...");
    bluetoothActivo = true;
    SerialBT.begin(config.nombre_equipo);
    menuBluetoothGeneral();
    return;
  }

  ciclos++;
  debugPrint("🔁 Ciclo #" + String(ciclos));
  debugPrint("📦 Firmware: " + String(FIRMWARE_VERSION));
  debugPrint("🔧 Equipo: " + String(config.nombre_equipo));
  debugPrint("⚙️ Iniciando sistema...");

  if (config.usar_modbus && !iniciarModbus()) return;

  actualizarEstado(ESTADO_OK);
}
void loop() {
  if (modoServidorWeb) {servidorWeb.handleClient();return;}

  const DateTime now = rtc.now();

  debugPrint("⏳ Leyendo sensores...");
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  if (isnan(tempC) || isnan(humidity)) {
    tempC = -99.9;
    humidity = -99.9;
  }

  int suelo_ads_S30 = leerPromedioADC(0, 5, 50, false);
  int suelo_ads_S15 = leerPromedioADC(1, 5, 50, false);
  int hS30 = calcularPorcentajeHumedad(suelo_ads_S30, config.sueloS30_max, config.sueloS30_min);
  int hS15 = calcularPorcentajeHumedad(suelo_ads_S15, config.sueloS15_max, config.sueloS15_min);

  String strS30 = (hS30 == -1) ? "SAT-H" : (hS30 == 101) ? "SAT-S" : String(hS30);
  String strS15 = (hS15 == -1) ? "SAT-H" : (hS15 == 101) ? "SAT-S" : String(hS15);

  int radiacion = config.usar_modbus ? leerRadiacionModbus() : -1;
  String radiacionStr = (radiacion < 0) ? "NA" : String(radiacion);

  sensors.requestTemperatures();
  float tempS1 = getTemp(config.tempSuelo1_addr);
  float tempS2 = getTemp(config.tempSuelo2_addr);

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
  guardarEnSD(lineaCSV);

  enviarDatos(lineaCSV);

  parpadearLed(LED_VERDE, 3, false);
  delay(500);

  digitalWrite(MODEM_PWR, LOW);
  digitalWrite(POWER_CTRL_PIN, LOW);
  WiFi.disconnect(true);

  const uint64_t tiempo_sleep_us = static_cast<uint64_t>(config.intervalo_minutos) * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(tiempo_sleep_us);
  debugPrint("💤 Entrando en sleep profundo por " + String(config.intervalo_minutos) + " minutos...");
  delay(100);
  ultimoEstado = estadoActual;
  esp_deep_sleep_start();
}

void editarConfiguracion() {
  while (true) {
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

    String opcion = leerLineaBluetooth();

    if (opcion == "1") {
      editarCampoConfig("nombre del equipo", config.nombre_equipo, sizeof(config.nombre_equipo));
    } else if (opcion == "2") {
      editarCampoNumConfig("intervalo de medición (min)", &config.intervalo_minutos);
    } else if (opcion == "3") {
      editarCampoConfig("número SMS DATOS", config.numero_SMSDatos, sizeof(config.numero_SMSDatos));
    } else if (opcion == "4") {
      editarCampoConfig("número SMS NOTIFICACIONES", config.numero_SMSNotif, sizeof(config.numero_SMSNotif));
    } else if (opcion == "5") {
      editarCampoNumConfig("reintentos de SMS", &config.reintentos_envio_sms);
    } else if (opcion == "6") {
      alternarCampoBool("SIM800L", &config.usar_sim800);
    } else if (opcion == "7") {
      alternarCampoBool("Modbus", &config.usar_modbus);
    } else if (opcion == "8") {
      alternarCampoBool("LEDs de estado", &config.usar_leds_estado);
    } else if (opcion == "9") {
      editarCampoNumConfig("Tolerancia ADC", &config.tolerancia_adc);
    } else if (opcion == "10") {
      editarDireccionSensor("Sensor 1", config.tempSuelo1_addr);
    } else if (opcion == "11") {
      editarDireccionSensor("Sensor 2", config.tempSuelo2_addr);
    } else if (opcion == "0") {
      SerialBT.println("🔙 Volviendo al menú principal...");
      break;
    } else {
      SerialBT.println("❌ Opción inválida. Intenta nuevamente.");
    }
  }
}
void enviarDatos(const String& datosCSV) {
  if (!sim800Inicializado) {
    debugPrint("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L();
  }
  if (!sim800Inicializado) {
    debugPrint("❌ No se pudo inicializar el módulo SIM para el envío de datos.");
    actualizarEstado(FALLO_NO_CRITICO, "SIM800L", "Fallo de inicio en envío");
  } else {
    if (!enviarSMS(config.numero_SMSDatos, datosCSV)) {
      actualizarEstado(FALLO_NO_CRITICO, "SMS_DATOS", "Fallo en envío de datos");
    }
  }
}
void editarDireccionSensor(const char* nombreSensor, uint8_t* destino) {
  SerialBT.println("✏️ Ingresa la nueva direccion para " + String(nombreSensor));
  SerialBT.println("   Formato: XX:XX:XX:XX:XX:XX:XX:XX");
  String entrada = leerLineaBluetooth();
  entrada.trim();

  int n_scanned = sscanf(entrada.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                         &destino[0], &destino[1], &destino[2], &destino[3],
                         &destino[4], &destino[5], &destino[6], &destino[7]);

  if (n_scanned == 8) {
    SerialBT.println("✅ Dirección actualizada correctamente.");
  } else {
    SerialBT.println("❌ Error: Formato incorrecto. No se guardaron los cambios.");
  }
}
void escanearDispositivosOneWire() {
  byte addr[8];

  SerialBT.println("\n🔍 Escaneando bus OneWire...");

  if (oneWire.search(addr)) {
    int count = 0;
    do {
      count++;
      SerialBT.print("  Dispositivo " + String(count) + ": ");

      for (int i = 0; i < 8; i++) {
        if (addr[i] < 16) {
          SerialBT.print("0");
        }
        SerialBT.print(addr[i], HEX);
        if (i < 7) {
          SerialBT.print(":");
        }
      }
      SerialBT.println();

      if (addr[0] == 0x28) {
        SerialBT.println("    -> (Sensor de temperatura DS18B20)");
      }

    } while (oneWire.search(addr));

    oneWire.reset_search();

  } else {
    SerialBT.println("❌ No se encontraron dispositivos en el bus OneWire.");
    oneWire.reset_search();
  }
}
void inspectorComandosAT() {
  SerialBT.println("\n🕵️ === Inspector de Comandos AT ===");
  SerialBT.println("Escribe un comando AT y presiona Enter para enviarlo al módem.");
  SerialBT.println("Escribe 'SALIR' para volver al menú principal.");
  SerialBT.println("----------------------------------------");

  // Asegurarse de que el módem esté encendido y la comunicación serial iniciada
  if (!sim800Inicializado) {
    SerialBT.println("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L();
  }
  if (!sim800Inicializado) {
    SerialBT.println("❌ No se pudo inicializar el módulo SIM. Volviendo al menú.");
    return;
  }

  while (true) {
    // Si se envía un comando desde el terminal Bluetooth
    if (SerialBT.available()) {
      String cmd = SerialBT.readStringUntil('\n');
      cmd.trim();

      if (cmd.equalsIgnoreCase("SALIR")) {
        SerialBT.println("🔙 Volviendo al menú de pruebas...");
        break;
      }

      SerialBT.println("> Enviando: " + cmd);
      modem.sendAT(cmd); // Envía el comando al módem

      String res = "";
      // Espera una respuesta del módem (con un tiempo límite de 2 segundos)
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
void mostrarConfiguracionActual() {
  DateTime now = rtc.now();
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  SerialBT.println("\n📄 ===== CONFIGURACIÓN ACTUAL DEL EQUIPO =====");
  SerialBT.println("Firmware: " + String(FIRMWARE_VERSION));
  SerialBT.println("Equipo: " + String(config.nombre_equipo));
  SerialBT.println("Num. Serie: " + String(config.numSerie));
  SerialBT.println("Intervalo de medición: " + String(config.intervalo_minutos) + " min");
  SerialBT.println("Hora actual del sistema: " + String(buf));

  SerialBT.println("\n--- Envio de datos y notificaciones SMS ---");
  SerialBT.println("Número DATOS: " + String(config.numero_SMSDatos));
  SerialBT.println("Número NOTIFICACIONES: " + String(config.numero_SMSNotif));
  SerialBT.println("Reintentos: " + String(config.reintentos_envio_sms));

  SerialBT.println("\n--- Módulos Adicionales ---");
  SerialBT.println("Modbus (Piranómetro): " + String(config.usar_modbus ? "Activado" : "Desactivado"));
  SerialBT.println("LEDs de estado: " + String(config.usar_leds_estado ? "Activados" : "Desactivados"));

  SerialBT.println("\n--- Calibración de Sensores ---");
  SerialBT.println("Tolerancia ADC: " + String(config.tolerancia_adc));
  SerialBT.println("Suelo S30 (Mojado/Seco): " + String(config.sueloS30_min) + " / " + String(config.sueloS30_max));
  SerialBT.println("Suelo S15 (Mojado/Seco): " + String(config.sueloS15_min) + " / " + String(config.sueloS15_max));

  char addr_buf[24];
  SerialBT.println("\n--- Direcciones de Sensores de Temperatura ---");
  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo1_addr[0], config.tempSuelo1_addr[1], config.tempSuelo1_addr[2], config.tempSuelo1_addr[3],
           config.tempSuelo1_addr[4], config.tempSuelo1_addr[5], config.tempSuelo1_addr[6], config.tempSuelo1_addr[7]);
  SerialBT.println("Sensor 1: " + String(addr_buf));
  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo2_addr[0], config.tempSuelo2_addr[1], config.tempSuelo2_addr[2], config.tempSuelo2_addr[3],
           config.tempSuelo2_addr[4], config.tempSuelo2_addr[5], config.tempSuelo2_addr[6], config.tempSuelo2_addr[7]);
  SerialBT.println("Sensor 2: " + String(addr_buf));

  SerialBT.println("✅ Fin de configuración.");
}
void menuPruebas() {
  while (true) {
    SerialBT.println("\n📶 Menú de pruebas:");
    SerialBT.println("1️⃣ Enviar datos de prueba mediante SMS 🌐");
    SerialBT.println("2️⃣ Enviar notificación mediante SMS 🔔");
    SerialBT.println("3️⃣ Probar sensores de temperatura de suelo 🌡");
    SerialBT.println("4️⃣ Probar sensores de humedad 🌱");
    SerialBT.println("5️⃣ Escanear bus OneWire 🔍");
    SerialBT.println("6️⃣ Inspector de Comandos AT 🕵️");
    SerialBT.println("0️⃣ Volver al menú principal 🔙");
    SerialBT.print("🔸 Elige una opción: ");

    String opcion = leerLineaBluetooth();
    opcion.toUpperCase();

    if (opcion == "1") probarEnvioDatos();
    else if (opcion == "2") probarEnvioNotif();
    else if (opcion == "3") probarSensoresTemperatura();
    else if (opcion == "4") probarSensoresHumedad();
    else if (opcion == "5") escanearDispositivosOneWire();
    else if (opcion == "6") inspectorComandosAT();
    else if (opcion == "0") break;
    else SerialBT.println("❌ Opción no válida.");
  }
}
void probarEnvioNotif(){
  if (!config.usar_sim800) {
    SerialBT.println("⚠️ El SIM800L está desactivado. Actívalo desde el menú.");
    return;
  }
  if (!sim800Inicializado) {
    SerialBT.println("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L();
  }
  if (!sim800Inicializado) {
    SerialBT.println("❌ No se pudo inicializar el módulo SIM. Abortando prueba.");
    return;
  }

  String mensajePruebaNotif = "Mensaje de prueba desde " + String(config.nombre_equipo);

  SerialBT.println("📲 Enviando SMS de notificacion al número " + String(config.numero_SMSNotif));
  SerialBT.println("   Contenido: " + mensajePruebaNotif);
  enviarSMS(config.numero_SMSNotif, mensajePruebaNotif);
}
bool enviarSMS(const char* numero, const String& mensaje) {
  if (!config.usar_sim800) {
    debugPrint("📛 Envio de SMS desactivado por configuracion.");
    return false;
  }
  if (!sim800Inicializado) {
    iniciarSIM800L();
  }

  for (int intento = 1; intento <= config.reintentos_envio_sms; intento++) {
    debugPrint("📤 Enviando SMS a " + String(numero) + " (intento " + String(intento) + ")...");

    if (modem.isNetworkConnected() && modem.sendSMS(numero, mensaje)) {
      debugPrint("✅ SMS enviado correctamente.");
      return true;
    } else {
      debugPrint("⚠️ Error al enviar SMS. Reiniciando modem...");
      digitalWrite(MODEM_PWR, LOW);
      delay(2000);
      digitalWrite(MODEM_PWR, HIGH);
      delay(2000);
      sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
      modem.restart();
      delay(3000);
    }

    delay(config.espera_entre_reintentos_sms_ms);
  }

  debugPrint("❌ No se pudo enviar el SMS despues de varios intentos.");
  return false;
}
void probarEnvioDatos(){
  if (!config.usar_sim800) {
    SerialBT.println("⚠️ El SIM800L está desactivado. Actívalo desde el menú.");
    return;
  }
  if (!sim800Inicializado) {
    SerialBT.println("📶 Módulo SIM apagado. Intentando iniciar...");
    sim800Inicializado = iniciarSIM800L();
  }
  if (!sim800Inicializado) {
    SerialBT.println("❌ No se pudo inicializar el módulo SIM. Abortando prueba.");
    return;
  }

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

  SerialBT.println("📲 Enviando SMS de prueba al número " + String(config.numero_SMSDatos));
  SerialBT.println("   Contenido: " + mensajeDePruebaCSV);
  enviarSMS(config.numero_SMSDatos, mensajeDePruebaCSV);
}
float getTemp(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return -127.0;
  }
  return tempC;
}
void probarSensoresHumedad() {
  SerialBT.println("\n🌱 Probando sensores de humedad del suelo...");

  if (!ads.begin()) {
    SerialBT.println("❌ Error: No se pudo inicializar el ADS1115.");
    return;
  }

  SerialBT.println("Lecturas en tiempo real (crudo / %):");

  int rawS30 = leerPromedioADC(0, 5, 50, false);
  int humS30 = calcularPorcentajeHumedad(rawS30, config.sueloS30_max, config.sueloS30_min);
  SerialBT.print("🔹 Sensor S30: " + String(rawS30) + " / ");
  if (humS30 == -1) {
    SerialBT.println("SAT-H (Saturado Húmedo)");
  } else if (humS30 == 101) {
    SerialBT.println("SAT-S (Saturado Seco)");
  } else {
    SerialBT.println(String(humS30) + "%");
  }

  int rawS15 = leerPromedioADC(1, 5, 50, false);
  int humS15 = calcularPorcentajeHumedad(rawS15, config.sueloS15_max, config.sueloS15_min);
  SerialBT.print("🔹 Sensor S15: " + String(rawS15) + " / ");
  if (humS15 == -1) {
    SerialBT.println("SAT-H (Saturado Húmedo)");
  } else if (humS15 == 101) {
    SerialBT.println("SAT-S (Saturado Seco)");
  } else {
    SerialBT.println(String(humS15) + "%");
  }
}
void probarSensoresTemperatura() {
  SerialBT.println("🌡️  Probando sensores de temperatura DS18B20...");
  sensors.begin();
  SerialBT.print("Solicitando lecturas...");
  sensors.requestTemperatures();
  SerialBT.println(" OK");

  // Leer y mostrar temperatura del sensor 1
  float tempS1 = getTemp(config.tempSuelo1_addr);
  SerialBT.print("🔹 Sensor de Suelo 1: ");
  if (tempS1 == DEVICE_DISCONNECTED_C) {
    SerialBT.println("❌ Error, sensor no encontrado.");
  } else {
    SerialBT.print(tempS1);
    SerialBT.println(" °C");
  }

  // Leer y mostrar temperatura del sensor 2
  float tempS2 = getTemp(config.tempSuelo2_addr);
  SerialBT.print("🔹 Sensor de Suelo 2: ");
  if (tempS2 == DEVICE_DISCONNECTED_C) {
    SerialBT.println("❌ Error, sensor no encontrado.");
  } else {
    SerialBT.print(tempS2);
    SerialBT.println(" °C");
  }
}
void iniciarServidorWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid_ap, password_ap);

  IPAddress ip = WiFi.softAPIP();
  debugPrint("📡 Punto de acceso iniciado en IP: " + ip.toString());

  servidorWeb.on("/", HTTP_GET, []() {
    File root = SD.open("/");
    String html = R"rawliteral(
      <!DOCTYPE html><html>
      <head>
        <meta charset='UTF-8'><title>HydroSense</title>
        <script>
          function eliminarArchivo(nombre) {
            if (confirm("¿Seguro que deseas eliminar " + nombre + "?")) {
              fetch('/delete?f=' + nombre)
                .then(res => res.text())
                .then(msg => alert(msg))
                .then(() => location.reload());
            }
          }
        </script>
      </head>
      <body style='font-family:sans-serif;text-align:center'>
        <p>
          <a href="/update">
            <button style="font-size:16px;padding:10px 20px;background-color:#007bff;color:white;border:none;border-radius:5px;">
              🔄 Actualizar Firmware OTA
            </button>
          </a>
        </p>
        <h2>📁 Archivos CSV disponibles</h2>
        <ul style='list-style:none;padding:0'>
    )rawliteral";

    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;

      String nombre = entry.name();
      if (nombre.endsWith(".csv")) {
        html += "<li style='margin:10px'>";
        html += "<form action='/log' method='get' style='display:inline'>";
        html += "<input type='hidden' name='f' value='" + nombre + "'>";
        html += "<button type='submit' style='font-size:16px;padding:6px 16px;'>📥 Descargar</button>";
        html += "</form> ";
        html += "<span style='font-size:16px;margin:0 10px'>" + nombre + "</span>";
        html += "<button style='font-size:16px;padding:6px 16px;color:red' onclick='eliminarArchivo(\"" + nombre + "\")'>❌ Eliminar</button>";
        html += "</li>";
      }
      entry.close();
    }

    servidorWeb.send(200, "text/html", html);
  });

  // Descarga de archivo
  servidorWeb.on("/log", HTTP_GET, []() {
    if (!servidorWeb.hasArg("f")) {
      servidorWeb.send(400, "text/plain", "❌ Parámetro de archivo no especificado");
      return;
    }

    String archivo = servidorWeb.arg("f");
    String path = "/" + archivo;

    if (!SD.exists(path)) {
      servidorWeb.send(404, "text/plain", "❌ Archivo no encontrado");
      return;
    }

    File f = SD.open(path, FILE_READ);
    if (!f) {
      servidorWeb.send(500, "text/plain", "❌ Error al abrir el archivo");
      return;
    }

    servidorWeb.sendHeader("Content-Type", "text/csv");
    servidorWeb.sendHeader("Content-Disposition", "attachment; filename=" + archivo);
    servidorWeb.sendHeader("Connection", "close");
    servidorWeb.streamFile(f, "text/csv");
    f.close();
  });

  // Eliminación de archivo
  servidorWeb.on("/delete", HTTP_GET, []() {
    if (!servidorWeb.hasArg("f")) {
      servidorWeb.send(400, "text/plain", "❌ Parámetro de archivo no especificado");
      return;
    }

    String archivo = servidorWeb.arg("f");
    String path = "/" + archivo;

    if (!SD.exists(path)) {
      servidorWeb.send(404, "text/plain", "❌ Archivo no encontrado");
      return;
    }

    if (SD.remove(path)) {
      servidorWeb.send(200, "text/plain", "✅ Archivo eliminado: " + archivo);
    } else {
      servidorWeb.send(500, "text/plain", "❌ Error al eliminar el archivo");
    }
  });

  // Página de actualización OTA
  servidorWeb.on("/update", HTTP_GET, []() {
    servidorWeb.send(200, "text/html", R"rawliteral(
    <!DOCTYPE html><html>
    <head><meta charset='UTF-8'><title>HydroSense OTA</title></head>
    <body style='font-family:sans-serif;text-align:center'>
      <h2>🚀 Actualización de Firmware OTA</h2>
      <form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' name='firmware' accept='.bin' required><br><br>
        <input type='submit' value='Actualizar'>
      </form>
      <p>⚠️ No apagues ni reinicies el dispositivo durante la actualización.</p>
    </body>
    </html>
  )rawliteral");
  });

  // POST del archivo binario (manejo OTA)
  servidorWeb.on("/update", HTTP_POST, []() {
    bool exito = !Update.hasError();
    servidorWeb.send(200, "text/plain",
      exito ? "[OK] Actualizacion exitosa. Reiniciando..." :
              "[FAIL] Fallo la actualizacion.");
    delay(1500);
    if (exito) ESP.restart();
  }, []() {
    HTTPUpload& upload = servidorWeb.upload();

    if (upload.status == UPLOAD_FILE_START) {
      debugPrint("⬆️ Iniciando carga OTA: " + upload.filename);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        debugPrint("✅ Firmware cargado y listo");
      } else {
        Update.printError(Serial);
      }
    }
  });


  servidorWeb.begin();
  debugPrint("🌐 Servidor iniciado. Conéctate a 'HydroSense-AP'");
}
String obtenerNombreLogSemanal(const DateTime& dt) {
  // Lunes = 1, Domingo = 7
  int diaSemana = dt.dayOfTheWeek();
  int diasARestar = (diaSemana == 0) ? 6 : diaSemana - 1;

  DateTime inicioSemana = dt - TimeSpan(diasARestar, 0, 0, 0);

  char nombre[32];
  snprintf(nombre, sizeof(nombre), "log_%04d-%02d-%02d.csv",
           inicioSemana.year(), inicioSemana.month(), inicioSemana.day());
  return String(nombre);
}
void guardarEnSD(const String& lineaCSV) {
  const DateTime now = rtc.now();
  const String nombreLog = obtenerNombreLogSemanal(now);
  const String path = "/" + nombreLog;
  const bool nuevoArchivo = !SD.exists(path);

  logFile = SD.open(path, FILE_APPEND);
  if (!logFile) {
    debugPrint("❌ Error al abrir archivo " + nombreLog);
    actualizarEstado(FALLO_CRITICO);
    return;
  }

  if (nuevoArchivo || logFile.size() == 0) {
    const String encabezado = "Equipo,Fecha y hora,Temp. ambiente [C],Humedad ambiente [%],Humedad S30 [%],Temp S1 [C],Valor ADC S30,Humedad S15 [%],Temp S2 [C],Valor ADC S15,Radiacion [W/m2]";
    logFile.println(encabezado);
  }

  if (!logFile.println(lineaCSV)) {
    debugPrint("❌ Error al escribir en " + nombreLog);
    actualizarEstado(FALLO_CRITICO);
  } else {
    debugPrint("📥 Datos guardados en " + nombreLog);
  }

  logFile.close();
}
void menuBluetoothGeneral() {
  while (!SerialBT.hasClient()) {
    debugPrint("⏳ Esperando cliente Bluetooth...");
    delay(500);
  }

  while (true) {
    SerialBT.println();
    SerialBT.println("📡 === MODO CONFIGURACIÓN BLUETOOTH ===");
    SerialBT.println("0️⃣ Configuración rápida de despliegue 🚀");
    SerialBT.println("1️⃣ Ver configuración actual");
    SerialBT.println("2️⃣ Editar parámetros");
    SerialBT.println("3️⃣ Calibrar sensores de humedad 🌱");
    SerialBT.println("4️⃣ Guardar configuración 💾");
    SerialBT.println("5️⃣ Sincronizar hora 🕒");
    SerialBT.println("6️⃣ Reiniciar equipo ♻️");
    SerialBT.println("7️⃣️ Menu de pruebas 📝");
    SerialBT.println("8️⃣ Salir del modo Bluetooth 🚪");
    SerialBT.println("🔸 Selecciona una opción: ");

    const int8_t opcion = leerEnteroBluetooth();

    switch (opcion) {
      case 0: configuracionRapidaDespliegue(); break;
      case 1: mostrarConfiguracionActual(); break;
      case 2: editarConfiguracion(); break;
      case 3: menuCalibracionSensores(); break;
      case 4: guardarConfigEnArchivo(); break;
      case 5: sincronizarHoraPorBluetooth(); break;
      case 6:
        SerialBT.println("♻️ Reiniciando...");
        delay(1000);
        ESP.restart();
        break;
      case 7: menuPruebas(); break;
      case 8:
        SerialBT.println("🚪 Saliendo del modo Bluetooth...");
        return;
      default:
        SerialBT.println("❌ Opción inválida. Intenta de nuevo.");
    }
  }
}
void configuracionRapidaDespliegue() {
  bool configModificada = false;
  SerialBT.println("\n🚀 Iniciando configuración rápida de despliegue...\n");

  // Mostrar información actual
  mostrarConfiguracionActual();

  SerialBT.println("\n¿Deseas cambiar el nombre del equipo? (s/n): ");
  String respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();
  if (respuesta == "s" || respuesta == "si") {
    editarCampoConfig("nombre del equipo", config.nombre_equipo, sizeof(config.nombre_equipo));
    configModificada = true;
  }

  SerialBT.println("\n¿Deseas actualizar la hora del sistema? (s/n): ");
  respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();
  if (respuesta == "s" || respuesta == "si") {
    sincronizarHoraPorBluetooth();
    configModificada = true;
  }

  SerialBT.println("\n¿Deseas iniciar el proceso de calibración de sensores de suelo? (s/n): ");
  respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();
  if (respuesta == "s" || respuesta == "si") {
    calibrarSensoresSuelo();
  }

  if (configModificada) {
    SerialBT.println("💾 Guardando configuración...");
    if (guardarConfigEnArchivo()) {
      SerialBT.println("✅ Configuración guardada exitosamente.");
    } else {
      SerialBT.println("❌ Error al guardar la configuración.");
    }
  }

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
void parpadearAmbosLeds(uint8_t cantidad) {

  for (uint8_t i = 0; i < cantidad; i++) {
    digitalWrite(LED_VERDE, HIGH);
    digitalWrite(LED_ROJO, HIGH);
    delay(500);
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_ROJO, LOW);
    delay(200);
  }
}
void detectarModoInicio() {
  pinMode(PIN_BLUETOOTH_ACTIVADOR, INPUT_PULLUP);

  if (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
    unsigned long t0 = millis();
    bool mostrado_5s = false;
    bool mostrado_10s = false;

    while (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
      unsigned long ahora = millis();
      unsigned long duracion = ahora - t0;

      if (duracion >= 5000 && !mostrado_5s) {
        parpadearAmbosLeds(3);
        mostrado_5s = true;
      }

      if (duracion >= 10000 && !mostrado_10s) {
        parpadearAmbosLeds(3);
        mostrado_10s = true;
      }

      delay(50);
    }

    unsigned long duracion = millis() - t0;

    if (duracion >= 10000) {
      modoBluetooth = false;
      modoServidorWeb = true;
      //debugPrint("Modo WiFi activo");
    } else if (duracion >= 5000) {
      modoBluetooth = true;
      modoServidorWeb = false;
      //debugPrint("Modo bluetooth activo");
    }
  }
}
bool sincronizarHoraPorBluetooth() {
  debugPrint("⏳ Envia la hora por Bluetooth en formato:");
  debugPrint("📤 YYYY-MM-DD HH:MM:SS");
  debugPrint("➡️ Ejemplo: 2025-07-06 18:45:00");

  String entrada = "";

  // 🔄 Espera indefinida
  while (entrada.length() < 19) {
    if (SerialBT.available()) {
      entrada = SerialBT.readStringUntil('\n');
      entrada.trim();
      debugPrint("📨 Recibido: " + entrada);
    }
    delay(100);  // Reducir carga de CPU
  }

  // ✅ Procesamiento
  int year   = entrada.substring(0,  4).toInt();
  int month  = entrada.substring(5,  7).toInt();
  int day    = entrada.substring(8, 10).toInt();
  int hour   = entrada.substring(11,13).toInt();
  int minute = entrada.substring(14,16).toInt();
  int second = entrada.substring(17,19).toInt();

  if (year < 2020 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 59) {
    debugPrint("❌ Fecha u hora fuera de rango.");
    return false;
      }

  rtc.adjust(DateTime(year, month, day, hour, minute, second));

  char buf[30];
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
           year, month, day, hour, minute, second);
  debugPrint("✅ RTC ajustado manualmente:");
  debugPrint(String(buf));

  return true;
}
bool iniciarSIM800L() {
  if (sim800Inicializado) {
    debugPrint("✅ SIM800L encendido");
  }else {
    debugPrint("📶 Encendiendo SIM800L...");
    digitalWrite(MODEM_PWR, HIGH);
    delay(3000);
    parpadearLed(LED_VERDE, 3, true);
    sim800Inicializado = true;
    debugPrint("✅ SIM800L encendido");
  }

  debugPrint("📡 Iniciando modem y buscando red...");
  sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  if (!modem.restart()) {
    debugPrint("❌ Fallo al reiniciar el módem");
    actualizarEstado(FALLO_NO_CRITICO, "SIM800L", "Reinicio fallido");
    sim800Inicializado = false;
    return false;
  }
  delay(3000);

  if (modem.waitForNetwork(config.timeout_red)) {
    debugPrint("✅ Red celular encontrada");
    parpadearLed(LED_VERDE, 3, true);
    int csq = modem.getSignalQuality();
    String operador = modem.getOperator();
    debugPrint("📶 Señal: " + String(csq) + " (0-31)");
    debugPrint("🏢 Operador: " + operador);
    sim800Inicializado = true;
    return true;
  }
  debugPrint("❌ Sin red celular disponible");
  actualizarEstado(FALLO_NO_CRITICO, "SIM800L", "Sin red celular");
  sim800Inicializado = false;
  return false;
}
bool iniciarRTC() {
  debugPrint("⏰ Inicializando RTC DS3231...");

  if (!rtc.begin()) {
    debugPrint("❌ RTC no detectado");
    actualizarEstado(FALLO_CRITICO, "RTC", "No se detecto el RTC DS3231");
    return false;
  }

  if (rtc.lostPower()) {
    debugPrint("⚠️ El RTC perdió la hora. Requiere ajuste manual o desde internet.");
  } else {
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
String fechaActual(const DateTime& dt) {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%04d/%02d/%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
  return String(buffer);
}
bool verificarModbus() {
  uint8_t result = node.readHoldingRegisters(0x0000, 1);
  if (result == node.ku8MBSuccess) {
    return true;
  } else {
    debugPrint("❌ Fallo Modbus. Codigo: " + String(result));
    return false;
  }
}
int leerRadiacionModbus() {
  uint8_t result = node.readHoldingRegisters(0x0000, 1);

  if (result == node.ku8MBSuccess) {
    int16_t valor = node.getResponseBuffer(0);

    if (valor >= 0 && valor <= 2000) {
      return valor;
    } else {
      debugPrint("⚠️ Valor de radiacion fuera de rango: " + String(valor));
      return -2;
    }
  } else {
    debugPrint("❌ Error en lectura Modbus. Codigo: " + String(result));
    return -1;
  }
}
void actualizarEstado(EstadoSistema estado, const String& id, const String& descripcion) {
  estadoActual = estado;
  ultimoEstado = estado;
  switch (estado) {
    case ESTADO_OK:
      digitalWrite(LED_ROJO, LOW);
      parpadearLed(LED_VERDE, 5, false);
      break;

    case FALLO_CRITICO: {
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROJO, HIGH);

      const String fechaHora = fechaActual(rtc.now());
      const String mensaje = "¡ERROR! Equipo: " + String(config.nombre_equipo) +
                              "\nFALLO CRITICO\nID: " + id +
                                "\nHora: " + fechaHora + "\n" +
                                  descripcion +
                                    "\nAccion: Modo sleep indefinido.";
      enviarSMS(config.numero_SMSNotif, mensaje);

      digitalWrite(MODEM_PWR, LOW);
      digitalWrite(POWER_CTRL_PIN, LOW);
      delay(500);

      esp_deep_sleep_start();
      break;
    }

    case FALLO_MEDIO: {
      digitalWrite(LED_VERDE, LOW);
      parpadearLed(LED_ROJO, 4, true);

      const String fechaHora = fechaActual(rtc.now());
      const String mensaje = "¡ERROR! Equipo: " + String(config.nombre_equipo) +
                              "\nFALLO MEDIO\nID: " + id +
                                "\nHora: " + fechaHora + "\n" +
                                  descripcion;
      enviarSMS(config.numero_SMSNotif, mensaje);
      break;
    }

    case FALLO_NO_CRITICO:
      digitalWrite(LED_VERDE, LOW);
      parpadearLed(LED_ROJO, 3, true);
      break;
  }
}
void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo) {
  const uint16_t onTime = largo ? 1000 : 300;
  const uint16_t offTime = largo ? 500 : 200;
  while (cantidad--) {
    digitalWrite(pinLed, HIGH);
    delay(onTime);
    digitalWrite(pinLed, LOW);
    delay(offTime);
  }
}
int leerPromedioADC(uint8_t canal,uint8_t muestras,uint16_t intervaloMs,bool debugBT) {
  int suma = 0;
  for (uint8_t i = 0; i < muestras; i++) {
    int valor = ads.readADC_SingleEnded(canal);
    suma += valor;
    // Si activaste Bluetooth y pediste debug, imprime cada lectura
    if (debugBT && bluetoothActivo && SerialBT.hasClient()) {
      SerialBT.println("📡 Lectura " + String(i+1) + "/" +
                       String(muestras) + ": " + String(valor));
    }
    delay(intervaloMs);
  }
  return suma / muestras;
}
void calibrarSensoresSuelo() {
  while (SerialBT.available()) SerialBT.read();

  if (!ads.begin()) {
    SerialBT.println("❌ ADS1115 no inicializado. Cancelando calibracion.");
    return;
  }

  SerialBT.println("🔧 Iniciando calibracion de sensores de suelo.");

  // === CALIBRACIoN S30 ===
  SerialBT.println("➡️ Coloca el sensor S30 en ambiente SECO y presiona ENTER.");
  while (!SerialBT.available()) delay(100);
  while (SerialBT.available()) SerialBT.read();  // limpiar buffer
  delay(500);
  int S30_seco = leerPromedioADC(0, 5, 1000, true);
  SerialBT.println("✅ Leido S30 seco: " + String(S30_seco));

  SerialBT.println("➡️ Ahora coloca S30 en ambiente MOJADO y presiona ENTER.");
  while (!SerialBT.available()) delay(100);
  while (SerialBT.available()) SerialBT.read();
  delay(500);
  int S30_mojado = leerPromedioADC(0, 5, 1000, true);
  SerialBT.println("✅ Leido S30 mojado: " + String(S30_mojado));

  // === CALIBRACIoN S15 ===
  SerialBT.println("➡️ Coloca el sensor S15 en ambiente SECO y presiona ENTER.");
  while (!SerialBT.available()) delay(100);
  while (SerialBT.available()) SerialBT.read();
  delay(500);
  int S15_seco = leerPromedioADC(1, 5, 1000, true);
  SerialBT.println("✅ Leido S15 seco: " + String(S15_seco));

  SerialBT.println("➡️ Ahora coloca S15 en ambiente MOJADO y presiona ENTER.");
  while (!SerialBT.available()) delay(100);
  while (SerialBT.available()) SerialBT.read();
  delay(500);
  int S15_mojado = leerPromedioADC(1, 5, 1000, true);
  SerialBT.println("✅ Leido S15 mojado: " + String(S15_mojado));

  // === ACTUALIZACIÓN EN RAM ===
  config.sueloS30_min = S30_mojado;
  config.sueloS30_max = S30_seco;
  config.sueloS15_min = S15_mojado;
  config.sueloS15_max = S15_seco;

  SerialBT.println("✅ Calibración almacenada en memoria RAM.");
  SerialBT.println("💾 Recuerda guardar la configuración si deseas hacerla permanente.");
}
void debugPrint(const String& mensaje) {
  Serial.println(mensaje);
  if (bluetoothActivo && SerialBT.hasClient()) {
    SerialBT.println(mensaje);
  }
}
bool iniciarSD() {
  debugPrint("💾 Inicializando tarjeta microSD...");
  if (!SD.begin(SD_CS)) {
    debugPrint("❌ Fallo al inicializar microSD");
    actualizarEstado(FALLO_CRITICO, "SD", "Fallo al inicializar la microSD");
    return false;
  }
  debugPrint("✅ microSD inicializada");
  return true;
}
bool iniciarADS1115() {
  debugPrint("🔍 Inicializando ADS1115...");
  if (!ads.begin()) {
    debugPrint("❌ Fallo al inicializar ADS1115");
    actualizarEstado(FALLO_CRITICO, "ADS1115", "Fallo al inicializar el ADC");
    return false;
  }
  debugPrint("✅ ADS1115 inicializado");
  return true;
}
String leerLineaBluetooth() {
  String entrada = "";
  while (true) {
    while (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '\n' || c == '\r') {
        if (entrada.length() > 0) return entrada;
      } else {
        entrada += c;
      }
    }
    delay(10);
  }
}
int leerEnteroBluetooth() {
  return leerLineaBluetooth().toInt();
}
void editarCampoConfig(const char* nombreCampo, char* destino, size_t maxLen) {
  SerialBT.printf("✏️ Ingresa nuevo valor para %s: ", nombreCampo);
  String entrada = leerLineaBluetooth();
  entrada.trim();
  entrada.toCharArray(destino, maxLen);
  SerialBT.println("✅ Valor actualizado.");
}
void editarCampoNumConfig(const char* nombreCampo, int* destino) {
  SerialBT.printf("✏️ Ingresa nuevo valor numérico para %s: ", nombreCampo);
  int valor = leerEnteroBluetooth();
  *destino = valor;
  SerialBT.println("✅ Valor actualizado.");
}
void alternarCampoBool(const char* nombreCampo, bool* destino) {
  *destino = !(*destino);
  SerialBT.printf("🔁 %s ahora está: %s\n", nombreCampo, *destino ? "✅ Activado" : "❌ Desactivado");
}
void menuCalibracionSensores() {
  while (true) {
    SerialBT.println("\n🌱 === CALIBRACIÓN DE SENSORES DE HUMEDAD ===");
    SerialBT.println("1️⃣ Iniciar calibración interactiva");
    SerialBT.println("2️⃣ Mostrar valores actuales de calibración");
    SerialBT.println("3️⃣ Volver al menú anterior 🔙");
    SerialBT.print("🔸 Selecciona una opción: ");

    String opcion = leerLineaBluetooth();

    if (opcion == "1") {
      SerialBT.println("🧪 Iniciando proceso de calibración...");
      calibrarSensoresSuelo();
      SerialBT.println("✅ Calibración finalizada.");
    } else if (opcion == "2") {
      SerialBT.println("📊 Valores actuales de calibración:");
      SerialBT.print("S30 seco (min): ");
      SerialBT.println(config.sueloS30_min);
      SerialBT.print("S30 mojado (max): ");
      SerialBT.println(config.sueloS30_max);
      SerialBT.print("S15 seco (min): ");
      SerialBT.println(config.sueloS15_min);
      SerialBT.print("S15 mojado (max): ");
      SerialBT.println(config.sueloS15_max);
    } else if (opcion == "3") {
      SerialBT.println("🔙 Volviendo al menú anterior...");
      break;
    } else {
      SerialBT.println("❌ Opción inválida.");
    }
  }
}
bool guardarConfigEnArchivo() {
  JsonDocument doc;
  doc["nombre_equipo"] = config.nombre_equipo;
  doc["numSerie"] = config.numSerie;
  doc["sueloS30_min"] = config.sueloS30_min;
  doc["sueloS30_max"] = config.sueloS30_max;
  doc["sueloS15_min"] = config.sueloS15_min;
  doc["sueloS15_max"] = config.sueloS15_max;
  doc["intervalo_minutos"] = config.intervalo_minutos;
  doc["numero_SMSDatos"] = config.numero_SMSDatos;
  doc["numero_SMSNotif"] = config.numero_SMSNotif;
  doc["usar_modbus"] = config.usar_modbus;
  doc["usar_sim800"] = config.usar_sim800;
  doc["usar_leds_estado"] = config.usar_leds_estado;
  doc["reintentos_envio_sms"] = config.reintentos_envio_sms;
  doc["tolerancia_adc"] = config.tolerancia_adc;
  char addr_buf[24];
  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo1_addr[0], config.tempSuelo1_addr[1], config.tempSuelo1_addr[2], config.tempSuelo1_addr[3],
           config.tempSuelo1_addr[4], config.tempSuelo1_addr[5], config.tempSuelo1_addr[6], config.tempSuelo1_addr[7]);
  doc["tempSuelo1_addr"] = addr_buf;

  snprintf(addr_buf, sizeof(addr_buf), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
           config.tempSuelo2_addr[0], config.tempSuelo2_addr[1], config.tempSuelo2_addr[2], config.tempSuelo2_addr[3],
           config.tempSuelo2_addr[4], config.tempSuelo2_addr[5], config.tempSuelo2_addr[6], config.tempSuelo2_addr[7]);
  doc["tempSuelo2_addr"] = addr_buf;

  File configFile = SD.open("/config.json", FILE_WRITE);
  if (!configFile) {
    debugPrint("❌ Error al abrir config.json para escritura.");
    return false;
  }
  if (serializeJson(doc, configFile) == 0) {
    debugPrint("❌ Error al escribir en config.json.");
    configFile.close();
    return false;
  }
  configFile.close();
  debugPrint("✅ Configuracion guardada.");
  return true;
}
bool cargarConfiguracion() {
  debugPrint("📂 Cargando configuracion desde config.json...");
  File configFile = SD.open("/config.json", FILE_READ);
  if (!configFile) {
    debugPrint("❌ No se encontro config.json, se usarán valores por defecto.");
    return false;
  }
  JsonDocument doc;
  if (deserializeJson(doc, configFile)) {
    debugPrint("❌ Error al parsear config.json");
    configFile.close();
    return false;
  }
  configFile.close();

  // --- Carga de Parámetros ---
  strncpy(config.nombre_equipo, doc["nombre_equipo"] | "HydroSense", sizeof(config.nombre_equipo));
  strncpy(config.numSerie, doc["numSerie"] | "SN-001", sizeof(config.numSerie));
  config.sueloS30_min = doc["sueloS30_min"] | 10000;
  config.sueloS30_max = doc["sueloS30_max"] | 25000;
  config.sueloS15_min = doc["sueloS15_min"] | 10000;
  config.sueloS15_max = doc["sueloS15_max"] | 25000;
  config.intervalo_minutos = doc["intervalo_minutos"] | 15;
  config.timeout_red = doc["timeout_red"] | 30000;
  strncpy(config.numero_SMSDatos, doc["numero_SMSDatos"] | "+521234567890", sizeof(config.numero_SMSDatos));
  strncpy(config.numero_SMSNotif, doc["numero_SMSNotif"] | "+521234567890", sizeof(config.numero_SMSNotif));
  strncpy(config.archivo_log, doc["archivo_log"] | "log.csv", sizeof(config.archivo_log));
  config.reintentos_envio_sms = doc["reintentos_envio_sms"] | 3;
  config.espera_entre_reintentos_sms_ms = doc["espera_entre_reintentos_sms_ms"] | 500;
  config.usar_leds_estado = doc["usar_leds_estado"] | true;
  config.usar_modbus = doc["usar_modbus"] | true;
  config.usar_sim800 = doc["usar_sim800"] | true;
  config.tolerancia_adc = doc["tolerancia_adc"] | 250;
  const char* addr1_str = doc["tempSuelo1_addr"] | "28:32:B0:87:00:2D:04:A4";
  const char* addr2_str = doc["tempSuelo2_addr"] | "28:DE:62:87:00:03:79:35";
  sscanf(addr1_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &config.tempSuelo1_addr[0], &config.tempSuelo1_addr[1], &config.tempSuelo1_addr[2], &config.tempSuelo1_addr[3],
         &config.tempSuelo1_addr[4], &config.tempSuelo1_addr[5], &config.tempSuelo1_addr[6], &config.tempSuelo1_addr[7]);
  sscanf(addr2_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
         &config.tempSuelo2_addr[0], &config.tempSuelo2_addr[1], &config.tempSuelo2_addr[2], &config.tempSuelo2_addr[3],
         &config.tempSuelo2_addr[4], &config.tempSuelo2_addr[5], &config.tempSuelo2_addr[6], &config.tempSuelo2_addr[7]);


  debugPrint("✅ Configuracion cargada.");
  return true;
}
int calcularPorcentajeHumedad(const int valor, const int seco, const int mojado) {
  if (seco == mojado) return 0;

  const int minimoPermitido = mojado - config.tolerancia_adc;
  const int maximoPermitido = seco + config.tolerancia_adc;
  if (valor < minimoPermitido) return -1;
  if (valor > maximoPermitido) return 101;

  float porcentaje = 100.0 * (seco - valor) / (seco - mojado);

  return constrain(round(porcentaje), 0, 100);
}
bool iniciarModbus() {
  debugPrint("🟡 Iniciando protocolo Modbus...");
  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);

  Serial2.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);

  node.begin(1, Serial2);
  node.preTransmission([]() {
    digitalWrite(RS485_DE_RE, HIGH);
  });
  node.postTransmission([]() {
    digitalWrite(RS485_DE_RE, LOW);
  });
  delay(200);
  parpadearLed(LED_VERDE, 3, true);
  debugPrint("✅ Modbus listo");

  debugPrint("🛠️ Verificando conexion Modbus...");
  if (!verificarModbus()) {
    actualizarEstado(FALLO_CRITICO, "MODBUS", "Fallo en la comunicacion RS485");
    return false;
  }

  parpadearLed(LED_VERDE, 3, true);
  debugPrint("📡 Modbus OK");
  return true;
}