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
#include <RTClib.h>
#include <Wire.h>
#include <Arduino.h>
#include "FS.h"
#include "SD.h"

// RTC DS3231
bool rtcInicializado = false;
#define SDA_PIN 15
#define SCL_PIN 4
RTC_DS3231 rtc;
const char* daysOfWeek[7] = { "Domingo", "Lunes", "Martes", "Miercoles", "Jueves", "Viernes", "Sabado" };

// Sensor SHT-10
#define DATA_PIN 16
#define CLOCK_PIN 17
SHT1x sht1x(DATA_PIN, CLOCK_PIN, SHT1x::Voltage::DC_3_3v);

// Micro SD
#define SD_CS 5
File logFile;

// ADC sensores suelo
Adafruit_ADS1115 ads;
#define TOLERANCIA_ADC 250

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
constexpr char phoneNumber[] = "+524427547302";
bool sim800Inicializado = false;

// MOSFET de control
#define MODEM_PWR 22
#define POWER_CTRL_PIN 21

// LEDs de estado
#define LED_VERDE 13
#define LED_ROJO 14

// Bluetooth
#define PIN_BLUETOOTH_ACTIVADOR 12
bool bluetoothActivo = false;
BluetoothSerial SerialBT;

// Variables para control del sistema
#define FIRMWARE_VERSION "v2.4"
enum EstadoSistema {
  ESTADO_OK,
  FALLO_CRITICO,
  FALLO_MEDIO,
  FALLO_NO_CRITICO
};
EstadoSistema estadoActual;
RTC_DATA_ATTR uint8_t ciclos = 0;
RTC_DATA_ATTR EstadoSistema ultimoEstado = ESTADO_OK;

// Configuracion del sistema
struct ConfiguracionSistema {
  String nombre_equipo;
  uint16_t sueloS30_min;
  uint16_t sueloS30_max;
  uint16_t sueloS15_min;
  uint16_t sueloS15_max;
  uint16_t intervalo_minutos;
  String apn;
  String usuario_apn;
  String contrasena_apn;
  String numero_SMS;
  bool usar_modbus;
  bool usar_sim800;
  uint16_t timeout_red;
  String archivo_log;
  uint8_t reintentos_envio_sms;
  uint16_t espera_entre_reintentos_sms_ms;
  String api_host;
  uint16_t api_puerto;
  String api_endpoint;
};
ConfiguracionSistema config;

// Prototipos
void actualizarEstado(EstadoSistema estado, const String& id = "", const String& descripcion = "");
int leerPromedioADC(uint8_t canal, uint8_t muestras = 5,uint16_t intervaloMs = 50, bool debugBT = false);
void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo);
int calcularPorcentajeHumedad(int valor, int seco, int mojado);
void enviarDatosAPI(const String &lineaCSV);
String fechaActual(const DateTime& dt);
void debugPrint(const String& mensaje);
bool enviarSMS(const String& mensaje);
void guardarEnSD(const String& linea);
bool sincronizarHoraDesdeInternet();
void calibrarSensoresSuelo();
bool cargarConfiguracion();
int leerRadiacionModbus();
bool verificarModbus();
bool iniciarSIM800L();
bool iniciarADS1115();
bool iniciarModbus();
bool conectarGPRS();
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

  // Verificar boton y activar Bluetooth antes que cualquier otra cosa
  if (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
    Serial.println("🔧 Activando Bluetooth...");
    bluetoothActivo = true;
    SerialBT.begin("ESP32_BT");

    // 🔄 Confirmacion visual: parpadeo rapido de ambos LEDs
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_VERDE, HIGH);
      digitalWrite(LED_ROJO, HIGH);
      delay(150);
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROJO, LOW);
      delay(150);
      digitalWrite(LED_VERDE, HIGH);
      digitalWrite(LED_ROJO, HIGH);
      delay(150);
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROJO, LOW);
      delay(150);
    }

    constexpr unsigned long tiempoEsperaConexion = 20000;
    unsigned long inicioConexion = millis();

    while (millis() - inicioConexion < tiempoEsperaConexion) {
      if (SerialBT.hasClient()) {

        Serial.println("✅ Cliente Bluetooth conectado");
        SerialBT.println("✅ Conexion establecida con ESP32");
        SerialBT.println("🔧 Bluetooth listo para depuracion");

        // Preguntar por calibracion
        SerialBT.println("¿Deseas calibrar sensores de suelo? (S/N) [RECOMENDADO]");
        SerialBT.println("⌛ Esperando respuesta...");

        bool respuestaValida = false;
        while (!respuestaValida) {
          if (SerialBT.available()) {
            char r = toupper(SerialBT.read());
            if (r == 'S') {
              calibrarSensoresSuelo();
              respuestaValida = true;
            } else if (r == 'N') {
              SerialBT.println("⏭️ Calibracion omitida.");
              respuestaValida = true;
            } else {
              SerialBT.println("❓ Respuesta no valida. Usa 'S' o 'N'.");
            }
          }
          delay(100);
        }

        // Preguntar por sincronizacion de hora
        if (config.usar_sim800) {
          SerialBT.println("🕒 ¿Deseas sincronizar la hora desde internet? (S/N) [RECOMENDADO]");
          SerialBT.println("🕒 Esperando respuesta...");

          while (SerialBT.available()) SerialBT.read();
          delay(200);
          bool respuestaSyncValida = false;
          while (!respuestaSyncValida) {
            if (SerialBT.available()) {
              char r = toupper(SerialBT.read());

              if (r == 'S') {
                // Asegurarse que el RTC este inicializado
                if (!rtcInicializado) {
                  if (!iniciarRTC()) {
                    SerialBT.println("❌ No se pudo iniciar el RTC. Cancelando sincronizacion.");
                    break;
                  }
                  rtcInicializado = true;
                }

                // Asegurarse que el SIM800L este encendido (si no lo esta ya)
                if (!sim800Inicializado) {
                  if (!iniciarSIM800L()) {
                    SerialBT.println("❌ No se pudo iniciar el SIM800L. Cancelando sincronizacion.");
                    break;
                  }
                  sim800Inicializado = true;
                }

                SerialBT.println("🌐 Iniciando sincronizacion de hora...");
                if (!sincronizarHoraDesdeInternet()) {
                  SerialBT.println("❌ Error al sincronizar hora.");
                }
                respuestaSyncValida = true;

              } else if (r == 'N') {
                SerialBT.println("⏭️ Sincronizacion omitida.");
                respuestaSyncValida = true;

              } else {
                SerialBT.println("❓ Respuesta no valida. Usa 'S' o 'N'.");
              }
            }
            delay(100);
          }
        }

        // Preguntar si desea editar el intervalo de sleep
        SerialBT.println("🕒 ¿Deseas editar el intervalo de sleep en minutos? (S/N)");
        bool respuestaIntervaloValida = false;

        while (SerialBT.available()) SerialBT.read();
        delay(200);

        while (!respuestaIntervaloValida) {
          if (SerialBT.available()) {
            char r = toupper(SerialBT.read());
            if (r == 'S') {
              bool valorValido = false;
              while (!valorValido) {
                SerialBT.println("✏️ Ingresa el nuevo valor (en minutos, entre 2 y 1440):");

                while (SerialBT.available()) SerialBT.read();  // limpiar buffer
                delay(200);

                String valorStr = "";
                while (true) {
                  if (SerialBT.available()) {
                    char c = SerialBT.read();
                    if (c == '\n' || c == '\r') break;
                    if (isDigit(c)) valorStr += c;
                  }
                  delay(10);
                }

                uint16_t nuevoIntervalo = valorStr.toInt();
                if (nuevoIntervalo >= 2 && nuevoIntervalo <= 1440) {
                  config.intervalo_minutos = nuevoIntervalo;

                  File configFile = SD.open("/config.json", FILE_READ);
                  if (!configFile) {
                    SerialBT.println("❌ Error al abrir config.json.");
                    return;
                  }

                  JsonDocument doc;
                  DeserializationError error = deserializeJson(doc, configFile);
                  configFile.close();

                  if (error) {
                    SerialBT.println("❌ Error al parsear JSON.");
                    return;
                  }

                  doc["intervalo_minutos"] = nuevoIntervalo;
                  SD.remove("/config.json");
                  configFile = SD.open("/config.json", FILE_WRITE);
                  if (!configFile) {
                    SerialBT.println("❌ No se pudo guardar nuevo intervalo.");
                    return;
                  }

                  serializeJsonPretty(doc, configFile);
                  configFile.close();

                  SerialBT.println("✅ Intervalo actualizado a " + String(nuevoIntervalo) + " min.");
                  valorValido = true;
                } else {
                  SerialBT.println("⚠️ Valor invalido. Debe estar entre 1 y 1440.");
                }
              }
              respuestaIntervaloValida = true;

            } else if (r == 'N') {
              SerialBT.println("⏭️ Intervalo de sleep no modificado.");
              respuestaIntervaloValida = true;

            } else {
              SerialBT.println("❓ Respuesta no valida. Usa 'S' o 'N'.");
            }
          }
          delay(100);
        }

        // Preguntar si desea editar el numero de telefono
        SerialBT.println("📱 ¿Deseas editar el numero de telefono para SMS? (S/N)");
        bool respuestaTelefonoValida = false;

        while (SerialBT.available()) SerialBT.read();
        delay(200);

        while (!respuestaTelefonoValida) {
          if (SerialBT.available()) {
            char r = toupper(SerialBT.read());
            if (r == 'S') {
              bool telefonoValido = false;
              while (!telefonoValido) {
                SerialBT.println("✏️ Ingresa los 10 digitos del numero (ej: 4421234567):");

                while (SerialBT.available()) SerialBT.read();
                delay(200);

                String numeroUsuario = "";
                while (true) {
                  if (SerialBT.available()) {
                    char c = SerialBT.read();
                    if (c == '\n' || c == '\r') break;
                    if (isDigit(c)) numeroUsuario += c;
                  }
                  delay(10);
                }

                if (numeroUsuario.length() == 10) {
                  String numeroCompleto = "+52" + numeroUsuario;
                  config.numero_SMS = numeroCompleto;

                  File configFile = SD.open("/config.json", FILE_READ);
                  if (!configFile) {
                    SerialBT.println("❌ Error al abrir config.json.");
                    return;
                  }

                  JsonDocument doc;
                  DeserializationError error = deserializeJson(doc, configFile);
                  configFile.close();

                  if (error) {
                    SerialBT.println("❌ Error al parsear JSON.");
                    return;
                  }

                  doc["numero_SMS"] = numeroCompleto;
                  SD.remove("/config.json");
                  configFile = SD.open("/config.json", FILE_WRITE);
                  if (!configFile) {
                    SerialBT.println("❌ No se pudo guardar nuevo numero.");
                    return;
                  }

                  serializeJsonPretty(doc, configFile);
                  configFile.close();

                  SerialBT.println("✅ Numero actualizado a " + numeroCompleto);
                  telefonoValido = true;
                } else {
                  SerialBT.println("⚠️ Numero invalido. Deben ser exactamente 10 digitos.");
                }
              }
              respuestaTelefonoValida = true;

            } else if (r == 'N') {
              SerialBT.println("⏭️ Numero de telefono no modificado.");
              respuestaTelefonoValida = true;

            } else {
              SerialBT.println("❓ Respuesta no valida. Usa 'S' o 'N'.");
            }
          }
          delay(100);
        }

        // Preguntar si desea editar el nombre del equipo
        SerialBT.println("🏷️ ¿Deseas editar el nombre del equipo? (S/N)");
        bool respuestaNombreValida = false;

        while (SerialBT.available()) SerialBT.read();
        delay(200);

        while (!respuestaNombreValida) {
          if (SerialBT.available()) {
            char r = toupper(SerialBT.read());
            if (r == 'S') {
              SerialBT.println("✏️ Ingresa el nuevo nombre del equipo:");

              while (SerialBT.available()) SerialBT.read();
              delay(200);

              String nuevoNombre = "";
              while (true) {
                if (SerialBT.available()) {
                  char c = SerialBT.read();
                  if (c == '\n' || c == '\r') break;
                  if (isPrintable(c)) nuevoNombre += c;
                }
                delay(10);
              }

              nuevoNombre.trim();
              if (nuevoNombre.length() >= 3 && nuevoNombre.length() <= 30) {
                config.nombre_equipo = nuevoNombre;

                File configFile = SD.open("/config.json", FILE_READ);
                if (!configFile) {
                  SerialBT.println("❌ Error al abrir config.json.");
                  return;
                }

                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, configFile);
                configFile.close();

                if (error) {
                  SerialBT.println("❌ Error al parsear JSON.");
                  return;
                }

                doc["nombre_equipo"] = nuevoNombre;
                SD.remove("/config.json");
                configFile = SD.open("/config.json", FILE_WRITE);
                if (!configFile) {
                  SerialBT.println("❌ No se pudo guardar nuevo nombre.");
                  return;
                }

                serializeJsonPretty(doc, configFile);
                configFile.close();

                SerialBT.println("✅ Nombre del equipo actualizado a " + nuevoNombre);
                respuestaNombreValida = true;
              } else {
                SerialBT.println("⚠️ Nombre invalido. Debe tener entre 3 y 30 caracteres.");
              }

            } else if (r == 'N') {
              SerialBT.println("⏭️ Nombre del equipo no modificado.");
              respuestaNombreValida = true;

            } else {
              SerialBT.println("❓ Respuesta no valida. Usa 'S' o 'N'.");
            }
          }
          delay(100);
        }

        // Preguntar si desea editar los datos de la API
        SerialBT.println("🌐 ¿Deseas editar los datos de la API? (S/N)");
        bool respuestaApiValida = false;

        while (SerialBT.available()) SerialBT.read();
        delay(200);

        while (!respuestaApiValida) {
          if (SerialBT.available()) {
            char r = toupper(SerialBT.read());
            if (r == 'S') {
              // Leer archivo
              File configFile = SD.open("/config.json", FILE_READ);
              if (!configFile) {
                SerialBT.println("❌ Error al abrir config.json.");
                return;
              }
              JsonDocument doc;
              DeserializationError error = deserializeJson(doc, configFile);
              configFile.close();

              if (error) {
                SerialBT.println("❌ Error al parsear JSON.");
                return;
              }

              // Host
              SerialBT.println("✏️ Ingresa el nuevo HOST (ej: api.midominio.com):");
              String nuevoHost = "";
              while (true) {
                if (SerialBT.available()) {
                  char c = SerialBT.read();
                  if (c == '\n' || c == '\r') break;
                  if (isPrintable(c)) nuevoHost += c;
                }
                delay(10);
              }
              nuevoHost.trim();
              doc["api_host"] = nuevoHost;

              // Puerto
              SerialBT.println("✏️ Ingresa el nuevo PUERTO (ej: 80):");
              String puertoStr = "";
              while (true) {
                if (SerialBT.available()) {
                  char c = SerialBT.read();
                  if (c == '\n' || c == '\r') break;
                  if (isDigit(c)) puertoStr += c;
                }
                delay(10);
              }
              doc["api_puerto"] = puertoStr.toInt();

              // Endpoint
              SerialBT.println("✏️ Ingresa el nuevo ENDPOINT (ej: /api/registro):");
              String nuevoEndpoint = "";
              while (true) {
                if (SerialBT.available()) {
                  char c = SerialBT.read();
                  if (c == '\n' || c == '\r') break;
                  if (isPrintable(c)) nuevoEndpoint += c;
                }
                delay(10);
              }
              nuevoEndpoint.trim();
              doc["api_endpoint"] = nuevoEndpoint;

              // Guardar archivo
              SD.remove("/config.json");
              configFile = SD.open("/config.json", FILE_WRITE);
              if (!configFile) {
                SerialBT.println("❌ No se pudo guardar nuevo archivo.");
                return;
              }
              serializeJsonPretty(doc, configFile);
              configFile.close();

              SerialBT.println("✅ Datos de API actualizados.");
              respuestaApiValida = true;

            } else if (r == 'N') {
              SerialBT.println("⏭️ Datos de API no modificados.");
              respuestaApiValida = true;

            } else {
              SerialBT.println("❓ Respuesta no valida. Usa 'S' o 'N'.");
            }
          }
          delay(100);
        }


        break;
      } else {
        Serial.println("⏳ Esperando conexion Bluetooth...");
      }
      delay(1000);
    }

    if (!SerialBT.hasClient()) {
      Serial.println("⚠️ No se conecto ningun cliente por Bluetooth");
      SerialBT.end();
      bluetoothActivo = false;
    }

  } else {
    Serial.println("❌ Bluetooth no activado");
  }

  // CONTADOR
  ciclos++;
  debugPrint("🔁 Ciclo #" + String(ciclos));
  debugPrint("📦 Firmware: " + String(FIRMWARE_VERSION));
  debugPrint("🔧 Equipo: " + config.nombre_equipo);
  debugPrint("⚙️ Iniciando sistema...");

  // Encender modulos alimentados por MOSFET
  digitalWrite(POWER_CTRL_PIN, HIGH);
  delay(500);
  parpadearLed(LED_VERDE, 3, true);

  // INICIALIZAR MODULOS
  if (config.usar_modbus && !iniciarModbus()) return;
  if (config.usar_sim800 && !sim800Inicializado) {
    sim800Inicializado = iniciarSIM800L();
    if (!sim800Inicializado) return;
  }
  if (!iniciarRTC()) return;

  actualizarEstado(ESTADO_OK);
}

void loop() {
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
  const String output = config.nombre_equipo + "," + fechaActual(now) + "," + String(tempC, 1) + "," + String(humidity, 1) + "%," +
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

int calcularPorcentajeHumedad(const int valor, const int seco, const int mojado) {
  if (seco == mojado) return 0;

  const int minimoPermitido = mojado - TOLERANCIA_ADC;
  const int maximoPermitido = seco + TOLERANCIA_ADC;

  if (valor < minimoPermitido) return -1;
  if (valor > maximoPermitido) return 101;

  const int porcentaje = map(valor, seco, mojado, 0, 100);
  return constrain(porcentaje, 0, 100);
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

bool iniciarSIM800L() {
  debugPrint("📶 Encendiendo SIM800L...");
  digitalWrite(MODEM_PWR, HIGH);
  delay(2000);
  parpadearLed(LED_VERDE, 3, true);
  debugPrint("✅ SIM800L encendido");

  debugPrint("📡 Iniciando modem y buscando red...");
  sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  modem.restart();
  delay(3000);

  if (modem.waitForNetwork(config.timeout_red)) {
    debugPrint("✅ Red celular encontrada");
    parpadearLed(LED_VERDE, 3, true);
    return true;
  } else {
    debugPrint("❌ Sin red celular disponible");
    actualizarEstado(FALLO_NO_CRITICO);
    return false;
  }
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

  config.nombre_equipo = doc["nombre_equipo"] | "ESP32";
  config.sueloS30_min = doc["sueloS30_min"] | 0;
  config.sueloS30_max = doc["sueloS30_max"] | 20000;
  config.sueloS15_min = doc["sueloS15_min"] | 0;
  config.sueloS15_max = doc["sueloS15_max"] | 20000;
  config.intervalo_minutos = doc["intervalo_minutos"] | 30;
  config.apn = doc["apn"] | "internet.itelcel.com";
  config.usuario_apn = doc["usuario_apn"] | "webgprs";
  config.contrasena_apn = doc["contrasena_apn"] | "webgprs2002";
  config.numero_SMS = doc["numero_SMS"] | "+520000000000";
  config.usar_modbus = doc["usar_modbus"] | true;
  config.usar_sim800 = doc["usar_sim800"] | true;
  config.timeout_red = doc["timeout_red"] | 20000;
  config.archivo_log = doc["archivo_log"] | "log.csv";
  config.reintentos_envio_sms = doc["reintentos_envio_sms"] | 3;
  config.espera_entre_reintentos_sms_ms = doc["espera_entre_reintentos_sms_ms"] | 1000;
  config.api_host = doc["api_host"] | "miapi.ejemplo.com";
  config.api_puerto = doc["api_puerto"] | 80;
  config.api_endpoint = doc["api_endpoint"] | "/api/registro";

  debugPrint("✅ Configuracion cargada:");
  debugPrint("🔹 Equipo: " + config.nombre_equipo);
  debugPrint("🔹 Intervalo: " + String(config.intervalo_minutos) + " min");
  debugPrint("🔹 APN: " + config.apn);
  return true;
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

bool iniciarRTC() {
  debugPrint("⏰ Inicializando RTC DS3231...");

  if (!rtc.begin()) {
    debugPrint("❌ RTC no detectado");
    actualizarEstado(FALLO_CRITICO, "RTC", "No se detecto el RTC DS3231");
    return false;
  }

  debugPrint("✅ RTC inicializado correctamente");
  return true;
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

  // === LECTURA Y PARSEO DEL ARCHIVO ACTUAL ===
  if (!SD.exists("/config.json")) {
    SerialBT.println("❌ No se encontro config.json para guardar la calibracion.");
    return;
  }

  File configFile = SD.open("/config.json", FILE_READ);
  if (!configFile) {
    SerialBT.println("❌ Error al abrir config.json para lectura.");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    SerialBT.println("❌ Error al parsear config.json: " + String(error.c_str()));
    return;
  }

  // === ACTUALIZACIoN DE VALORES ===
  doc["sueloS30_min"] = S30_mojado;
  doc["sueloS30_max"] = S30_seco;
  doc["sueloS15_min"] = S15_mojado;
  doc["sueloS15_max"] = S15_seco;

  // === BORRAR Y REESCRIBIR EL ARCHIVO DE CONFIGURACIoN ===
  SD.remove("/config.json");  // eliminar version anterior

  configFile = SD.open("/config.json", FILE_WRITE);
  if (!configFile) {
    SerialBT.println("❌ No se pudo crear config.json para escritura.");
    return;
  }

  if (serializeJsonPretty(doc, configFile) == 0) {
    SerialBT.println("❌ Error al escribir JSON en config.json.");
  } else {
    SerialBT.println("✅ Calibracion guardada exitosamente en config.json.");
  }

  configFile.close();
}

void debugPrint(const String& mensaje) {
  Serial.println(mensaje);
  if (bluetoothActivo && SerialBT.hasClient()) {
    SerialBT.println(mensaje);
  }
}

bool enviarSMS(const String& mensaje) {
  if (!config.usar_sim800) {
    debugPrint("📛 Envio de SMS desactivado por configuracion.");
    return false;
  }

  for (int intento = 1; intento <= config.reintentos_envio_sms; intento++) {
    debugPrint("📤 Enviando SMS (intento " + String(intento) + ")...");

    if (modem.isNetworkConnected() && modem.sendSMS(config.numero_SMS.c_str(), mensaje)) {
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


  debugPrint("No se pudo enviar el SMS despues de varios intentos.");
  return false;
}

bool sincronizarHoraDesdeInternet() {
  if (!config.usar_sim800 || !modem.isNetworkConnected()) {
    debugPrint("📛 No hay red disponible para sincronizar hora.");
    return false;
  }

  const auto host = "worldtimeapi.org";
  constexpr int port = 80;
  const auto endpoint = "/api/ip";

  debugPrint("🌐 Conectando a worldtimeapi.org...");
  if (!gsmClient.connect(host, port)) {
    debugPrint("❌ No se pudo conectar al servidor de hora.");
    return false;
  }

  gsmClient.print(String("GET ") + endpoint + " HTTP/1.1\r\n" +
                  "Host: " + host + "\r\n" +
                  "Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (gsmClient.connected() && !gsmClient.available() && millis() - timeout < 10000) {
    delay(100);
  }

  while (gsmClient.available()) {
    String linea = gsmClient.readStringUntil('\n');
    if (linea.startsWith("{")) {
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, linea);
      if (error) {
        debugPrint("❌ Error al analizar JSON de hora.");
        gsmClient.stop();
        return false;
      }

      String datetime = doc["datetime"];  // "2025-07-03T04:27:55.123456-06:00"
      int year = datetime.substring(0, 4).toInt();
      int month = datetime.substring(5, 7).toInt();
      int day = datetime.substring(8, 10).toInt();
      int hour = datetime.substring(11, 13).toInt();
      int minute = datetime.substring(14, 16).toInt();
      int second = datetime.substring(17, 19).toInt();

      rtc.adjust(DateTime(year, month, day, hour, minute, second));
      debugPrint("⏰ Hora sincronizada desde internet:");
      debugPrint(datetime);
      gsmClient.stop();
      return true;
    }
  }

  gsmClient.stop();
  return false;
}

bool conectarGPRS() {
  debugPrint("Conectando a red celular...");
  if (!modem.waitForNetwork(30000)) {
    debugPrint("No hay red.");
    return false;
  }

  if (!modem.gprsConnect(config.apn.c_str(), config.usuario_apn.c_str(), config.contrasena_apn.c_str())) {
    debugPrint("Error al conectar APN.");
    return false;
  }


  if (!modem.isNetworkConnected()) {
    debugPrint("APN aparentemente conectado, pero sin red real.");
    return false;
  }

  debugPrint("Red GPRS conectada.");
  return true;
}

String fechaActual(const DateTime& dt) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "%04d/%02d/%02d (%s) %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           daysOfWeek[dt.dayOfTheWeek()],
           dt.hour(), dt.minute(), dt.second());
  return String(buffer);
}

void guardarEnSD(const String& linea) {
  const bool nuevoArchivo = !SD.exists("/" + config.archivo_log);

  logFile = SD.open("/" + config.archivo_log, FILE_APPEND);
  if (!logFile) {
    debugPrint("❌ Error al abrir log.csv");
    actualizarEstado(FALLO_CRITICO);
    return;
  }

  // Si es nuevo archivo o estaba vacio, agregar encabezado
  if (nuevoArchivo || logFile.size() == 0) {
    const String encabezado = "Equipo,Fecha y hora,Temp[°C],Humedad[%],Humedad A0,Humedad A1,Radiacion[W/m2]";
    logFile.println(encabezado);
  }

  if (!logFile.println(linea)) {
    debugPrint("❌ Error al escribir en log.csv");
    actualizarEstado(FALLO_CRITICO);
  } else {
    debugPrint("📥 Datos guardados en " + config.archivo_log);
  }

  logFile.close();
}

void enviarDatosAPI(const String &lineaCSV) {
  if (!config.usar_sim800 || !modem.isNetworkConnected()) {
    debugPrint("📛 No hay red disponible o SIM800 desactivado.");
    return;
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
  doc["humedad_suelo_1"] = campos[4];
  doc["humedad_suelo_2"] = campos[5];
  doc["radiacion"] = campos[6];

  String json;
  serializeJson(doc, json);

  const String host = config.api_host;
  const int port = config.api_puerto;
  const String endpoint = config.api_endpoint;

  debugPrint("🌐 Conectando a " + host + ":" + String(port));
  if (!gsmClient.connect(host.c_str(), port)) {
    debugPrint("❌ No se pudo conectar al servidor");
    return;
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
      const String mensaje = "Equipo: " + config.nombre_equipo + "\nFALLO CRITICO\nID: " + id + "\nHora: " + fechaHora + "\n" +
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
      const String mensaje = "Equipo: " + config.nombre_equipo + "\nFALLO MEDIO\nID: " + id + "\nHora: " + fechaHora + "\n" + descripcion;
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
