/*
PIN LIST:
  + NOT TO USE:
    - D0
    - D1
    - D2
    - D3

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
    - Verde 13
  
  + Bluetooth
    - Activador 12
*/

#define TINY_GSM_MODEM_SIM800
#include <Adafruit_ADS1X15.h>
#include <BluetoothSerial.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <SHT1x-ESP.h>
#include <WebServer.h>
#include <Arduino.h>
#include <Update.h>
#include <RTClib.h>
#include <Wire.h>
#include <WiFi.h>
#include <FS.h>
#include <SD.h>

// RTC DS3231
bool rtcInicializado = false;
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
#define TOLERANCIA_ADC 250
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
TinyGsmClient gsmClient(modem);
bool sim800Inicializado = false;
constexpr char phoneNumber[] = "+524427547302";

// MOSFET de control
#define POWER_CTRL_PIN 21
#define MODEM_PWR 22

// LEDs de estado
#define LED_VERDE 13
#define LED_ROJO 14

// Bluetooth
#define PIN_BLUETOOTH_ACTIVADOR 12
bool bluetoothActivo = false;
BluetoothSerial SerialBT;

// WebServer
const char* ssid_ap = "HydroSense-AP";
const char* password_ap = "hidrosense";
WebServer servidorWeb(80);

// Variables para control del sistema
#define FIRMWARE_VERSION "v2.6"
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
  char nombre_equipo[32];
  char numSerie[16];
  int sueloS30_min;
  int sueloS30_max;
  int sueloS15_min;
  int sueloS15_max;
  int intervalo_minutos;
  char apn[64];
  char usuario_apn[32];
  char contrasena_apn[32];
  char numero_SMS[20];
  bool usar_modbus;
  bool usar_sim800;
  bool usar_leds_estado;
  uint16_t timeout_red;
  char archivo_log[32];
  int reintentos_envio_sms;
  int espera_entre_reintentos_sms_ms;
  char api_host[64];
  char api_endpoint[64];
  int api_puerto;
};
ConfiguracionSistema config;

// Prototipos
int leerPromedioADC(uint8_t canal, uint8_t muestras = 5,uint16_t intervaloMs = 50, bool debugBT = false);
void actualizarEstado(EstadoSistema estado, const String& id = "", const String& descripcion = "");
void editarCampoConfig(const char* nombreCampo, char* destino, size_t maxLen);
void editarCampoNumConfig(const char* nombreCampo, int* destino);
void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo);
int calcularPorcentajeHumedad(int valor, int seco, int mojado);
void alternarCampoBool(const char* nombreCampo, bool* destino);
void parpadearAmbosLeds(uint8_t cantidad, bool largo);
bool enviarDatosAPI(const String &lineaCSV);
String fechaActual(const DateTime& dt);
void debugPrint(const String& mensaje);
bool enviarSMS(const String& mensaje);
void guardarEnSD(const String& linea);
void configuracionRapidaDespliegue();
bool sincronizarHoraPorBluetooth();
void mostrarConfiguracionActual();
String obtenerNombreLogSemanal();
void menuCalibracionSensores();
bool guardarConfigEnArchivo();
void calibrarSensoresSuelo();
void menuBluetoothGeneral();
String leerLineaBluetooth();
void editarConfiguracion();
bool cargarConfiguracion();
int leerRadiacionModbus();
int leerEnteroBluetooth();
void iniciarServidorWeb();
void manejarDescargaLog();
void detectarModoInicio();
bool verificarModbus();
void probarEnvioAPI();
void probarEnvioSMS();
bool iniciarSIM800L();
bool iniciarADS1115();
bool iniciarModbus();
bool conectarGPRS();
void menuPruebas();
bool iniciarRTC();
bool iniciarSD();

void setup() {
  delay(5000);

  // Pines iniciales
  pinMode(PIN_BLUETOOTH_ACTIVADOR, INPUT_PULLUP);
  pinMode(POWER_CTRL_PIN, OUTPUT);
  pinMode(MODEM_PWR, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_ROJO, LOW);

  // I2C
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  // Serial
  Serial.begin(115200);
  delay(2000);

  // Inicializar modulos necesarios para calibracion
  digitalWrite(POWER_CTRL_PIN, HIGH); // Enciende SD, ADS1115, RS485
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

  // CONTADOR
  ciclos++;
  debugPrint("🔁 Ciclo #" + String(ciclos));
  debugPrint("📦 Firmware: " + String(FIRMWARE_VERSION));
  debugPrint("🔧 Equipo: " + String(config.nombre_equipo));
  debugPrint("⚙️ Iniciando sistema...");

  // INICIALIZAR MODULOS
  if (config.usar_modbus && !iniciarModbus()) return;
  if (config.usar_sim800 && !sim800Inicializado) {
    sim800Inicializado = iniciarSIM800L();
    if (!sim800Inicializado) return;
  }

  actualizarEstado(ESTADO_OK);
}
void loop() {
  if (modoServidorWeb) {
    servidorWeb.handleClient();
    return;
  }

  // Obteniendo fecha
  const DateTime now = rtc.now();

  // Leer el SHT-10
  debugPrint("⏳ Leyendo SHT10...");
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  if (isnan(tempC) || isnan(humidity) || tempC < -40 || tempC > 80) {
    debugPrint("⚠️ Error al leer sensor SHT10");
    actualizarEstado(FALLO_MEDIO);
  }

  // Leer sensores de suelo
  debugPrint("⏳ Leyendo sensores suelo...");
  int suelo_ads_S30 = leerPromedioADC(0);  // 5 muestras, 50 ms, sin debug
  int suelo_ads_S15 = leerPromedioADC(1);

  int hS30 = calcularPorcentajeHumedad(suelo_ads_S30, config.sueloS30_max, config.sueloS30_min);
  int hS15 = calcularPorcentajeHumedad(suelo_ads_S15, config.sueloS15_max, config.sueloS15_min);

  String strS30, strS15;

  // Revisar saturacion de humedad y actualizar estado si aplica

  if (hS30 == -1) {
    strS30 = "SAT-H";
    actualizarEstado(FALLO_MEDIO, "SATURACION_H", "Exceso de humedad detectado en sensor S30.");
  } else if (hS30 == 101) {
    strS30 = "SAT-S";
    actualizarEstado(FALLO_MEDIO, "SATURACION_S", "Sequedad extrema detectada en sensor S30.");
  } else {
    strS30 = String(hS30) + "%";
  }

  if (hS15 == -1) {
    strS15 = "SAT-H";
    actualizarEstado(FALLO_MEDIO, "SATURACION_H", "Exceso de humedad detectado en sensor S15.");
  } else if (hS15 == 101) {
    strS15 = "SAT-S";
    actualizarEstado(FALLO_MEDIO, "SATURACION_S", "Sequedad extrema detectada en sensor S15.");
  } else {
    strS15 = String(hS15) + "%";
  }

  // Leer radiacion
  debugPrint("⏳ Leyendo piranometro...");
  int radiacion = config.usar_modbus ? leerRadiacionModbus() : -1;
  bool radiacionErronea = (radiacion < 0);
  String radiacionStr = radiacionErronea ? "NA" : String(radiacion);

  // Actualizar estado si hay error
  if (radiacionErronea) {
    actualizarEstado(FALLO_MEDIO, "RADIACION", "Valor invalido de radiacion solar leido.");
  }

  // Crear linea de datos
  debugPrint("📊 Datos obtenidos...");
  const String output = String(config.nombre_equipo) + "," + fechaActual(now) + "," + String(tempC, 1) + "," + String(humidity, 1) + "%," +
                strS30 + "," + strS15 + "," + radiacionStr;
  debugPrint(output);
  guardarEnSD(output);
  enviarDatosAPI(output);
  parpadearLed(LED_VERDE, 3, false);

  delay(500);

  // Apagar modulos
  digitalWrite(MODEM_PWR, LOW);
  digitalWrite(POWER_CTRL_PIN, LOW);

  // Configurar timer
  const uint64_t tiempo_sleep_us = static_cast<uint64_t>(config.intervalo_minutos) * 60ULL * 1000000ULL;
  esp_sleep_enable_timer_wakeup(tiempo_sleep_us);
  debugPrint("💤 Entrando en sleep profundo por " + String(config.intervalo_minutos) + " minutos...");
  delay(100);
  ultimoEstado = estadoActual;
  esp_deep_sleep_start();
}

// Testing
void probarEnvioAPI() {
  if (!config.usar_sim800) {
    SerialBT.println("⚠️ El SIM800L está desactivado. Actívalo desde el menú de configuración.");
    return;
  }

  if (!sim800Inicializado) {
    SerialBT.println("📶 Iniciando SIM800L para envío de prueba...");
    sim800Inicializado = iniciarSIM800L();

    if (!sim800Inicializado) {
      SerialBT.println("❌ No se pudo inicializar el SIM800L.");
      return;
    }
  }

  SerialBT.println("🌐 Enviando datos de prueba a la API...");
  const String testLine = String(config.nombre_equipo) + ",pruebaFecha,0.0, 0.0, 0.0, 0.0, 0";
  const bool exito = enviarDatosAPI(testLine);

  if (exito) {
    SerialBT.println("✅ Envío exitoso.");
  } else {
    SerialBT.println("❌ Error en el envío de datos.");
  }
}
bool enviarDatosAPI(const String &lineaCSV) {
  if (!modem.isNetworkConnected() || !config.usar_sim800) {
    debugPrint("📛 No hay red disponible o SIM800 desactivado.");
    return false;
  }

  JsonDocument doc;
  int index = 0;
  String campos[7];
  for (int i = 0; i < 7; i++) {
    int sep = lineaCSV.indexOf(",", index);
    if (sep == -1) sep = lineaCSV.length();
    campos[i] = lineaCSV.substring(index, sep);
    index = sep + 1;
  }

  doc["equipo"] = campos[0];
  doc["fecha"] = campos[1];
  doc["temperatura"] = campos[2].toFloat();
  doc["humedad_relativa"] = campos[3];
  doc["humedad_suelo_s30"] = campos[4];
  doc["humedad_suelo_s15"] = campos[5];
  doc["radiacion"] = campos[6];

  String json;
  serializeJson(doc, json);

  const String host = config.api_host;
  const int port = config.api_puerto;
  const String endpoint = config.api_endpoint;

  debugPrint("🌐 Conectando a " + host + ":" + String(port));
  if (!gsmClient.connect(host.c_str(), port)) {
    debugPrint("❌ No se pudo conectar al servidor");
    return false;
  }

  String request =
    "POST " + endpoint + " HTTP/1.1\r\n" +
    "Host: " + host + "\r\n" +
    "Content-Type: application/json\r\n" +
    "Connection: close\r\n" +
    "Content-Length: " + String(json.length()) + "\r\n\r\n" +
    json;

  gsmClient.print(request);

  unsigned long timeout = millis();
  while (gsmClient.connected() && millis() - timeout < 10000) {
    if (gsmClient.available()) {
      String line = gsmClient.readStringUntil('\n');
      if (line.startsWith("HTTP/1.1 200")) {
        debugPrint("✅ Datos enviados correctamente.");
      } else if (line.startsWith("HTTP/1.1")) {
        debugPrint("⚠️ Respuesta del servidor: " + line);
      }
    }
  }

  gsmClient.stop();
  return true;
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


// Terminadas
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
void guardarEnSD(const String& linea) {
  // Obtener nombre del archivo basado en el lunes de esta semana
  const DateTime now = rtc.now();
  const String nombreLog = obtenerNombreLogSemanal(now);
  strncpy(config.archivo_log, nombreLog.c_str(), sizeof(config.archivo_log)); // solo en RAM

  const String path = "/" + nombreLog;
  const bool nuevoArchivo = !SD.exists(path);

  logFile = SD.open(path, FILE_APPEND);
  if (!logFile) {
    debugPrint("❌ Error al abrir archivo " + nombreLog);
    actualizarEstado(FALLO_CRITICO);
    return;
  }

  // Encabezado solo si es nuevo o estaba vacío
  if (nuevoArchivo || logFile.size() == 0) {
    const String encabezado = "Equipo,Fecha y hora,Temp. ambiente [C],Humedad ambiente [%],Humedad S30,Humedad S15,Radiacion [W/m2]";
    logFile.println(encabezado);
  }

  // Escribir datos
  if (!logFile.println(linea)) {
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
  SerialBT.println("\n🚀 Iniciando configuración rápida de despliegue...\n");

  // Cambiar nombre del equipo
  editarCampoConfig("nombre del equipo", config.nombre_equipo, sizeof(config.nombre_equipo));

  // Sincronizar hora (si SIM800 está activo)
  sincronizarHoraPorBluetooth();

  // Calibración de sensores
  calibrarSensoresSuelo();

  // Guardar configuración
  SerialBT.println("💾 Guardando configuración...");
  if (guardarConfigEnArchivo()) {
    SerialBT.println("✅ Configuración guardada exitosamente.");
  } else {
    SerialBT.println("❌ Error al guardar la configuración.");
  }

  // Mensaje final
  SerialBT.println("✅ Configuración rápida completada y guardada.");
  SerialBT.println("Ya puedes instalar el equipo 🚀 ¿Deseas reiniciar el sistema ahora? (s/n): ");
  String respuesta = leerLineaBluetooth();
  respuesta.toLowerCase();

  if (respuesta == "s" || respuesta == "si") {
    SerialBT.println("🔄 Reiniciando sistema...");
    delay(1000);
    ESP.restart();
  } else {
    SerialBT.println("⏹️ Reinicio cancelado. Puedes continuar configurando el sistema.");
  }
}
void manejarDescargaLog() {
  String path = "/" + String(config.archivo_log);
  if (!SD.exists(path)) {
    servidorWeb.send(404, "text/plain", "❌ Archivo no encontrado");
    return;
  }

  File archivo = SD.open(path, FILE_READ);
  if (!archivo) {
    servidorWeb.send(500, "text/plain", "❌ Error al abrir archivo");
    return;
  }

  servidorWeb.sendHeader("Content-Type", "text/csv");
  servidorWeb.sendHeader("Content-Disposition", "attachment; filename=log.csv");
  servidorWeb.sendHeader("Connection", "close");
  servidorWeb.streamFile(archivo, "text/csv");
  archivo.close();
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
void probarEnvioSMS() {
  if (!config.usar_sim800) {
    SerialBT.println("⚠️ El SIM800L está desactivado. Actívalo desde el menú de configuración.");
    return;
  }

  if (!sim800Inicializado) {
    SerialBT.println("📶 Iniciando SIM800L para envío de SMS...");
    sim800Inicializado = iniciarSIM800L();

    if (!sim800Inicializado) {
      SerialBT.println("❌ No se pudo inicializar el SIM800L.");
      return;
    }
  }

  const String mensaje = "SMS de prueba desde " + String(config.nombre_equipo);
  SerialBT.println("📲 Enviando SMS de prueba al número " + String(config.numero_SMS));

  enviarSMS(mensaje);
}
bool enviarSMS(const String& mensaje) {
  if (!config.usar_sim800) {
    debugPrint("📛 Envio de SMS desactivado por configuracion.");
    return false;
  }
  if (!sim800Inicializado) {
    iniciarSIM800L();
  }

  for (int intento = 1; intento <= config.reintentos_envio_sms; intento++) {
    debugPrint("📤 Enviando SMS (intento " + String(intento) + ")...");

    if (modem.isNetworkConnected() && modem.sendSMS(config.numero_SMS, mensaje)) {
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
    // Información adicional para diagnóstico
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
bool conectarGPRS() {
  debugPrint("🌐 Conectando GPRS...");
  if (!modem.waitForNetwork(20000)) {
    debugPrint("❌ No hay red.");
    return false;
  }

  if (!modem.gprsConnect(config.apn, config.usuario_apn, config.contrasena_apn)) {
    debugPrint("❌ Error al conectar APN.");
    return false;
  }

  if (!modem.isNetworkConnected()) {
    debugPrint("❌ APN aparentemente conectado, pero sin red real.");
    return false;
  }

  debugPrint("📡 Red conectada.");
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

    case FALLO_CRITICO:{
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROJO, HIGH);

      const String fechaHora = fechaActual(rtc.now());
      debugPrint("Fallo critico detectado. El sistema pasara a modo sleep hasta revision.");
      const String mensaje = "Equipo: " + String(config.nombre_equipo) + "\nFALLO CRITICO\nID: " + id + "\nHora: " + fechaHora + "\n" +
                       descripcion + "\nAccion: Modo sleep indefinido.";
      enviarSMS(mensaje);

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
      const String mensaje = "Equipo: " + String(config.nombre_equipo) + "\nFALLO MEDIO\nID: " + id + "\nHora: " + fechaHora + "\n" + descripcion;
      enviarSMS(mensaje);
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
void mostrarConfiguracionActual() {
  SerialBT.println("📄 ===== CONFIGURACIÓN ACTUAL =====");

  SerialBT.print("🔹 Nombre del equipo: ");
  SerialBT.println(config.nombre_equipo);

  SerialBT.print("🔹 Número de serie: ");
  SerialBT.println(config.numSerie);

  SerialBT.print("⏱️ Intervalo de medición: ");
  SerialBT.print(config.intervalo_minutos);
  SerialBT.println(" min");

  SerialBT.print("🌐 APN: ");
  SerialBT.println(config.apn);

  SerialBT.print("👤 Usuario APN: ");
  SerialBT.println(config.usuario_apn);

  SerialBT.print("🔐 Contraseña APN: ");
  SerialBT.println(config.contrasena_apn);

  SerialBT.print("📞 Número SMS: ");
  SerialBT.println(config.numero_SMS);

  SerialBT.print("📶 SIM800L: ");
  SerialBT.println(config.usar_sim800 ? "✅ Activado" : "❌ Desactivado");

  SerialBT.print("🔌 Modbus (piranómetro): ");
  SerialBT.println(config.usar_modbus ? "✅ Activado" : "❌ Desactivado");

  SerialBT.print("💡 LEDs de estado: ");
  SerialBT.println(config.usar_leds_estado ? "✅ Activados" : "❌ Apagados");

  SerialBT.print("📝 Archivo log: ");
  SerialBT.println(config.archivo_log);

  SerialBT.print("📤 Reintentos SMS: ");
  SerialBT.print(config.reintentos_envio_sms);
  SerialBT.print(" / Espera: ");
  SerialBT.print(config.espera_entre_reintentos_sms_ms);
  SerialBT.println(" ms");

  SerialBT.print("🌍 API Host: ");
  SerialBT.println(config.api_host);

  SerialBT.print("🔌 API Puerto: ");
  SerialBT.println(config.api_puerto);

  SerialBT.print("📍 API Endpoint: ");
  SerialBT.println(config.api_endpoint);

  SerialBT.print("🌱 Suelo S30 min/max: ");
  SerialBT.print(config.sueloS30_min);
  SerialBT.print(" / ");
  SerialBT.println(config.sueloS30_max);

  SerialBT.print("🌿 Suelo S15 min/max: ");
  SerialBT.print(config.sueloS15_min);
  SerialBT.print(" / ");
  SerialBT.println(config.sueloS15_max);

  // Mostrar hora actual del RTC
  DateTime now = rtc.now();
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  debugPrint("🕒 Hora actual del sistema: " + String(buf));

  SerialBT.println("✅ Fin de configuración.");
}
void editarConfiguracion() {
  while (true) {
    SerialBT.println("\n⚙️ === EDITAR CONFIGURACIÓN ===");
    SerialBT.println("1️⃣ Cambiar nombre del equipo");
    SerialBT.println("2️⃣ Cambiar intervalo de medición (min)");
    SerialBT.println("3️⃣ Configurar APN 🌐");
    SerialBT.println("4️⃣ Configurar número de teléfono SMS 📞");
    SerialBT.println("5️⃣ Activar/Desactivar SIM800L 📶");
    SerialBT.println("6️⃣ Activar/Desactivar Modbus 🔌");
    SerialBT.println("7️⃣ Activar/Desactivar LEDs de estado 💡");
    SerialBT.println("8️⃣ Configurar servidor API 🌍");
    SerialBT.println("9️⃣ Configurar reintentos de SMS");
    SerialBT.println("0️⃣ Volver al menú principal 🔙");
    SerialBT.print("🔸 Selecciona una opción: ");

    String opcion = leerLineaBluetooth();

    if (opcion == "1") {
      editarCampoConfig("nombre del equipo", config.nombre_equipo, sizeof(config.nombre_equipo));
    } else if (opcion == "2") {
      editarCampoNumConfig("intervalo de medición (min)", &config.intervalo_minutos);
    } else if (opcion == "3") {
      editarCampoConfig("APN", config.apn, sizeof(config.apn));
      editarCampoConfig("usuario APN", config.usuario_apn, sizeof(config.usuario_apn));
      editarCampoConfig("contraseña APN", config.contrasena_apn, sizeof(config.contrasena_apn));
    } else if (opcion == "4") {
      editarCampoConfig("número SMS", config.numero_SMS, sizeof(config.numero_SMS));
    } else if (opcion == "5") {
      alternarCampoBool("SIM800L", &config.usar_sim800);
    } else if (opcion == "6") {
      alternarCampoBool("Modbus", &config.usar_modbus);
    } else if (opcion == "7") {
      alternarCampoBool("LEDs de estado", &config.usar_leds_estado);
    } else if (opcion == "8") {
      editarCampoConfig("API host", config.api_host, sizeof(config.api_host));
      editarCampoConfig("API endpoint", config.api_endpoint, sizeof(config.api_endpoint));
      editarCampoNumConfig("API puerto", &config.api_puerto);
    } else if (opcion == "9") {
      editarCampoNumConfig("reintentos de SMS", &config.reintentos_envio_sms);
      editarCampoNumConfig("espera entre reintentos (ms)", &config.espera_entre_reintentos_sms_ms);
    } else if (opcion == "0") {
      SerialBT.println("🔙 Volviendo al menú principal...");
      break;
    } else {
      SerialBT.println("❌ Opción inválida. Intenta nuevamente.");
    }
  }
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

  // Copiar todos los campos del struct config al objeto JSON
  doc["nombre_equipo"] = config.nombre_equipo;
  doc["numSerie"] = config.numSerie;  // No editable, pero se conserva

  doc["sueloS30_min"] = config.sueloS30_min;
  doc["sueloS30_max"] = config.sueloS30_max;
  doc["sueloS15_min"] = config.sueloS15_min;
  doc["sueloS15_max"] = config.sueloS15_max;

  doc["intervalo_minutos"] = config.intervalo_minutos;

  doc["apn"] = config.apn;
  doc["usuario_apn"] = config.usuario_apn;
  doc["contrasena_apn"] = config.contrasena_apn;
  doc["numero_SMS"] = config.numero_SMS;

  doc["usar_modbus"] = config.usar_modbus;
  doc["usar_sim800"] = config.usar_sim800;
  doc["usar_leds_estado"] = config.usar_leds_estado;

  doc["archivo_log"] = config.archivo_log;
  doc["reintentos_envio_sms"] = config.reintentos_envio_sms;
  doc["espera_entre_reintentos_sms_ms"] = config.espera_entre_reintentos_sms_ms;

  doc["api_host"] = config.api_host;
  doc["api_puerto"] = config.api_puerto;
  doc["api_endpoint"] = config.api_endpoint;

  // Escribir en archivo
  SD.remove("/config.json");
  File configFile = SD.open("/config.json", FILE_WRITE);
  if (!configFile) {
    SerialBT.println("❌ Error al abrir config.json para escritura.");
    return false;
  }

  if (serializeJsonPretty(doc, configFile) == 0) {
    SerialBT.println("❌ Error al escribir JSON en config.json.");
    return false;
  }

  SerialBT.println("✅ Configuración guardada exitosamente en config.json.");
  configFile.close();
  return true;
}
bool cargarConfiguracion() {
  debugPrint("📂 Cargando configuracion desde config.json...");

  if (!SD.exists("/config.json")) {
    debugPrint("❌ No se encontro config.json");
    return false;
  }

  File configFile = SD.open("/config.json", FILE_READ);
  if (!configFile) {
    debugPrint("❌ No se pudo abrir config.json");
    return false;
  }

  JsonDocument doc;
  const DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    debugPrint("❌ Error al parsear config.json: " + String(error.c_str()));
    return false;
  }

  // Copiar campos tipo texto (char[]) usando strncpy
  strncpy(config.nombre_equipo, doc["nombre_equipo"] | "ESP32", sizeof(config.nombre_equipo));
  strncpy(config.numSerie, doc["numSerie"] | "CDHS000", sizeof(config.numSerie));
  strncpy(config.apn, doc["apn"] | "internet.itelcel.com", sizeof(config.apn));
  strncpy(config.usuario_apn, doc["usuario_apn"] | "webgprs", sizeof(config.usuario_apn));
  strncpy(config.contrasena_apn, doc["contrasena_apn"] | "webgprs2002", sizeof(config.contrasena_apn));
  strncpy(config.numero_SMS, doc["numero_SMS"] | "+520000000000", sizeof(config.numero_SMS));
  strncpy(config.archivo_log, doc["archivo_log"] | "log.csv", sizeof(config.archivo_log));
  strncpy(config.api_host, doc["api_host"] | "miapi.ejemplo.com", sizeof(config.api_host));
  strncpy(config.api_endpoint, doc["api_endpoint"] | "/api/registro", sizeof(config.api_endpoint));

  // Copiar campos numéricos
  config.sueloS30_min = doc["sueloS30_min"] | 0;
  config.sueloS30_max = doc["sueloS30_max"] | 20000;
  config.sueloS15_min = doc["sueloS15_min"] | 0;
  config.sueloS15_max = doc["sueloS15_max"] | 20000;
  config.intervalo_minutos = doc["intervalo_minutos"] | 30;
  config.usar_modbus = doc["usar_modbus"] | true;
  config.usar_sim800 = doc["usar_sim800"] | true;
  config.usar_leds_estado = doc["usar_leds_estado"] | true;
  config.timeout_red = doc["timeout_red"] | 20000;
  config.reintentos_envio_sms = doc["reintentos_envio_sms"] | 3;
  config.espera_entre_reintentos_sms_ms = doc["espera_entre_reintentos_sms_ms"] | 1000;
  config.api_puerto = doc["api_puerto"] | 80;

  // Mensaje de depuración
  debugPrint("✅ Configuracion cargada:");
  debugPrint("🔹 Equipo: " + String(config.nombre_equipo));
  debugPrint("🔹 Intervalo: " + String(config.intervalo_minutos) + " min");
  debugPrint("🔹 APN: " + String(config.apn));
  return true;
}
int calcularPorcentajeHumedad(const int valor, const int seco, const int mojado) {
  if (seco == mojado) return 0;

  const int minimoPermitido = mojado - TOLERANCIA_ADC;
  const int maximoPermitido = seco + TOLERANCIA_ADC;

  if (valor < minimoPermitido) return -1;
  if (valor > maximoPermitido) return 101;

  float porcentaje = 100.0 * (valor - mojado) / (seco - mojado);
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
void menuPruebas() {
  while (true) {
    SerialBT.println("\n📶 Menú de pruebas:");
    SerialBT.println("1️⃣ Enviar datos de prueba a la API 🌐");
    SerialBT.println("2️⃣ Enviar SMS de prueba 📲");
    SerialBT.println("0️⃣ Volver al menú principal 🔙");
    SerialBT.print("🔸 Elige una opción: ");

    String opcion = leerLineaBluetooth();

    if (opcion == "1") {
      probarEnvioAPI();
    } else if (opcion == "2") {
      probarEnvioSMS();
    } else if (opcion == "0") {
      SerialBT.println("🔙 Volviendo al menú principal...");
      break;
    } else {
      SerialBT.println("❌ Opción no válida.");
    }
  }
}