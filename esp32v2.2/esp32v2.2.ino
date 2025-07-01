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

#include <ArduinoHttpClient.h>
#include <Adafruit_ADS1X15.h> 
#define TINY_GSM_MODEM_SIM800
#include <BluetoothSerial.h>
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <SHT1x-ESP.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// RTC DS3231
#define SDA_PIN 15
#define SCL_PIN 4
RTC_DS3231 rtc;
const char* daysOfWeek[7] = { "Domingo", "Lunes", "Martes", "Miércoles", "Jueves", "Viernes", "Sábado" };

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
TinyGsmClient gsmClient(modem);
const char phoneNumber[] = "+524427547302";

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
RTC_NOINIT_ATTR int falloCriticoPrevio = 0;
#define FIRMWARE_VERSION "v2.1"
enum EstadoSistema {
  ESTADO_OK,
  FALLO_CRITICO,
  FALLO_MEDIO,
  FALLO_NO_CRITICO
};
EstadoSistema estadoActual;
RTC_DATA_ATTR uint32_t ciclos = 0;
RTC_DATA_ATTR EstadoSistema ultimoEstado = ESTADO_OK;

// Prototipos
void actualizarEstado(EstadoSistema estado, const String& id = "", const String& descripcion = "");
void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo);
String fechaActual(const DateTime& dt);
bool enviarSMS(const String& mensaje);
void guardarEnSD(const String& linea);
bool sincronizarHoraDesdeInternet();
void mostrarNivelDeSenal();
int leerRadiacionModbus();
void activarBluetooth();
void setupRS485Modbus();
bool verificarModbus();
bool conectarGPRS();
void debugPrint();

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

  // Activa bluetooth para debug
  if (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
    Serial.println("🔧 Activando Bluetooth...");
    bluetoothActivo = true;
    SerialBT.begin("ESP32_BT");

    const unsigned long tiempoEspera = 20000;
    unsigned long inicio = millis();

    while (millis() - inicio < tiempoEspera) {
      if (SerialBT.hasClient()) {
        Serial.println("✅ Cliente Bluetooth conectado");
        SerialBT.println("✅ Conexión establecida con ESP32");
        SerialBT.println("🔧 Bluetooth listo para depuración");
        break;
      } else {
        Serial.println("⏳ Esperando conexión Bluetooth...");
      }
      delay(1000);
    }

    if (!SerialBT.hasClient()) {
      Serial.println("⚠️ No se conectó ningún cliente por Bluetooth");
      SerialBT.end();
      bluetoothActivo = false;
    }
  } else {
    Serial.println("❌ Bluetooth no activado");
  } 

  ciclos++;
  debugPrint("🔁 Ciclo de operación #" + String(ciclos));
  debugPrint("⚙️ Iniciando sistema...");
  debugPrint("📦 Firmware: " + String(FIRMWARE_VERSION));
  delay(500);

  // Enciende micro SD, ADS1115 y MAX485
  debugPrint("🔌 Encendiendo módulos (micro SD, ADS1115, MAX485) ...");
  digitalWrite(POWER_CTRL_PIN, HIGH); 
  delay(500);
  parpadearLed(LED_VERDE, 3, true);
  debugPrint("✅ Listo");
  delay(200);

  debugPrint("🟡 Iniciando protocolo Modbus...");
  setupRS485Modbus();
  delay(200);
  parpadearLed(LED_VERDE, 3, true);
  debugPrint("✅ Listo");
  delay(200);

  debugPrint("🛠️ Verificando conexión Modbus...");
  if (!verificarModbus()) {
    actualizarEstado(FALLO_CRITICO, "MODBUS", "Fallo en la comunicación RS485");
  } else {
    parpadearLed(LED_VERDE, 3, true);
    debugPrint("📡 Modbus OK");
    delay(200);
  }

  // ADC sensores suelo
  debugPrint("🌱 Iniciando sensores de suelo (ADS1115)...");
  if (!ads.begin()) {
    debugPrint("❌ ADS1115 no detectado");
    actualizarEstado(FALLO_CRITICO, "ADC", "Modulo ADS1115 no detectado");
  } else {
    parpadearLed(LED_VERDE, 3, true);
    delay(200);
    debugPrint("✅ ADS1115 inicializado");
    delay(200);
  }

  // SD
  debugPrint("💾 Iniciando microSD...");
  if (!SD.begin(SD_CS)) {
    debugPrint("❌ Error al montar microSD");
    actualizarEstado(FALLO_CRITICO, "SD", "Error al iniciar tarjeta SD");
  } else {
    parpadearLed(LED_VERDE, 3, true);
    delay(200);
    debugPrint("✅ microSD lista");
    delay(200);

    debugPrint("🛠️ Verificando archivo de datos ...");
    if (SD.exists("/log.csv")) {
      logFile = SD.open("/log.csv", FILE_READ);
      String firstLine = logFile.readStringUntil('\n');
      logFile.close();
      if (!firstLine.startsWith("Fecha,")) {
        debugPrint("⚠️ Archivo log.csv corrupto o sin encabezado. Recreando.");
        SD.remove("/log.csv");
      }
    }

    if (!SD.exists("/log.csv")) {
      debugPrint("📝 Creando archivo CSV...");
      logFile = SD.open("/log.csv", FILE_WRITE);
      if (logFile) {
        logFile.println("Fecha,Temp (C),Humedad ambiente (%),Suelo A0 (ADC),Suelo A1 (ADC),Radiación solar (W/m2)");
        logFile.close();
        debugPrint("✅ Archivo CSV creado");
      }
    }
  }

  // Iniciar SIM800L
  debugPrint("📶 Encendiendo SIM800L...");
  digitalWrite(MODEM_PWR, HIGH);
  delay(2000);
  parpadearLed(LED_VERDE, 3, true);
  debugPrint("✅ Listo");
  delay(200);

  debugPrint("📡 Iniciando módem y buscando red...");
  sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  modem.restart();
  delay(3000);

  if (modem.waitForNetwork(20000)) {
    debugPrint("✅ Red celular encontrada");
    mostrarNivelDeSenal();
    parpadearLed(LED_VERDE, 3, true);
    delay(200);
  } else {
    debugPrint("❌ Sin red celular");
    actualizarEstado(FALLO_NO_CRITICO);
  }

  // RTC
  debugPrint("🕒 Iniciando RTC...");
  if (!rtc.begin()) {
    debugPrint("❌ RTC no detectado");
    actualizarEstado(FALLO_CRITICO, "RTC", "No se detecta el RTC");
  } else {
    parpadearLed(LED_VERDE, 3, true);
    delay(200);
    debugPrint("✅ RTC listo");
    delay(200);
  }

  if (rtc.lostPower() || ciclos % 48 == 0) {
    debugPrint("🌐 RTC sin hora o actualización periódica. Ciclo actual: " + String(ciclos));
    debugPrint("🔄 Sincronizando con el servidor...");
    if (!sincronizarHoraDesdeInternet()) {
      debugPrint("⚠️ No se pudo sincronizar la hora. Se usará hora de compilacion");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    } else {
      parpadearLed(LED_VERDE, 3, true);
      delay(200);
      debugPrint("✅ Hora actualizada correctamente");
      delay(200);
    }
  }

  actualizarEstado(ESTADO_OK);
}

void loop() {
  // Obteniendo fecha
  DateTime now = rtc.now();

  // Leer el SHT-10
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  if (isnan(tempC) || isnan(humidity) || tempC < -40 || tempC > 80) {
    debugPrint("⚠️ Error al leer sensor SHT10");
    actualizarEstado(FALLO_MEDIO);
  }
  
  // Leer sensores de suelo
  int suelo_ads_a0 = ads.readADC_SingleEnded(0);
  int suelo_ads_a1 = ads.readADC_SingleEnded(1);

  // Leer el piranómetro
  int radiacion = leerRadiacionModbus();
  String radiacionStr = (radiacion >= 0) ? String(radiacion) : "NA";

  // Crear linea de datos
  String output = fechaActual(rtc.now()) + "," + String(tempC, 1) + "," + String(humidity, 1) + "," +
                  String(suelo_ads_a0) + "," + String(suelo_ads_a1) + "," + radiacionStr;
  debugPrint(output);
  guardarEnSD(output);
  parpadearLed(LED_VERDE, 3, false);

  delay(500);

  // Apagar modulos
  digitalWrite(MODEM_PWR, LOW);
  digitalWrite(POWER_CTRL_PIN, LOW);

  // Configurar timer de 30 minutos
  esp_sleep_enable_timer_wakeup(2 * 60 * 1000000ULL);
  debugPrint("💤 Entrando en modo de sueño profundo por 2 minutos...");
  delay(100);
  ultimoEstado = estadoActual;
  esp_deep_sleep_start();
}

void debugPrint(const String& mensaje) {
  Serial.println(mensaje);
  if (bluetoothActivo && SerialBT.hasClient()) {
    SerialBT.println(mensaje);
  }
}

void activarBluetooth() {
  pinMode(PIN_BLUETOOTH_ACTIVADOR, INPUT_PULLUP);

  delay(100);

  if (digitalRead(PIN_BLUETOOTH_ACTIVADOR) == LOW) {
    debugPrint("🔧 Activando modo Bluetooth...");
    SerialBT.begin("ESP32_BT");
    bluetoothActivo = true;
    SerialBT.println("🔧 Modo depuración Bluetooth activo");
  } else {
    debugPrint("Bluetooth no activado");
  }
}

bool enviarSMS(const String& mensaje) {
  const int intentosMaximos = 3;

  for (int intento = 1; intento <= intentosMaximos; intento++) {
    debugPrint("Intentando enviar SMS (intento " + String(intento) + ")...");

    // Parpadear LED rojo rápido 5 veces
    parpadearLed(LED_ROJO, 5, false);

    // Verificar red y enviar
    if (modem.isNetworkConnected() && modem.sendSMS(phoneNumber, mensaje)) {
      debugPrint("SMS enviado correctamente.");
      return true;
    } else {
      debugPrint("Error al enviar SMS. Reiniciando módem...");

      // Apagar y encender el módem
      digitalWrite(MODEM_PWR, LOW);
      delay(2000);
      digitalWrite(MODEM_PWR, HIGH);
      delay(2000);

      sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
      modem.restart();
      delay(3000);
    }
  }

  debugPrint("No se pudo enviar el SMS después de varios intentos.");
  return false;
}

bool sincronizarHoraDesdeInternet() {
  if (!conectarGPRS()) return false;

  delay(3000);
  HttpClient http(gsmClient, "worldtimeapi.org", 80);
  http.get("/api/timezone/America/Mexico_City");

  int statusCode = http.responseStatusCode();
  if (statusCode != 200) {
    debugPrint("❌ Error HTTP: " + String(statusCode));
    return false;
  }

  String response = http.responseBody();
  debugPrint("Respuesta correcta del servidor.");

  // Parsear JSON
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    debugPrint("❌ Error al parsear JSON: " + String(error.c_str()));
    return false;
  }

  const char* datetime = doc["datetime"];
  int año, mes, dia, hora, minuto, segundo;
  sscanf(datetime, "%d-%d-%dT%d:%d:%d", &año, &mes, &dia, &hora, &minuto, &segundo);

  rtc.adjust(DateTime(año, mes, dia, hora, minuto, segundo));
  debugPrint("Hora sincronizada correctamente.");
  return true;
}

void mostrarNivelDeSenal() {
  int signal = modem.getSignalQuality();
  if (signal == 99) {
    debugPrint("Sin señal (CSQ: 99)");
  } else {
    int dBm = -113 + (signal * 2);
    debugPrint("Señal GSM: CSQ=" + String(signal) + " (~" + String(dBm) + " dBm)");
  }
}


bool conectarGPRS() {
  debugPrint("Conectando a red celular...");
  if (!modem.waitForNetwork(30000)) {
    debugPrint("No hay red.");
    return false;
  }

  if (!modem.gprsConnect("internet.itelcel.com", "webgprs", "webgprs2002")) {
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
  logFile = SD.open("/log.csv", FILE_APPEND);
  if (!logFile) {
    debugPrint("Error al abrir log.csv para escritura.");
    actualizarEstado(FALLO_MEDIO);
    return;
  }
  logFile.println(linea);
  logFile.close();
  debugPrint("Datos guardados");
}

void setupRS485Modbus() {
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
}

bool verificarModbus() {
  uint8_t result = node.readHoldingRegisters(0x0000, 1);
  if (result == node.ku8MBSuccess) {
    return true;
  } else {
    debugPrint("❌ Fallo Modbus. Código: " + String(result));
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
      debugPrint("⚠️ Valor de radiación fuera de rango: " + String(valor));
      return -2;
    }
  } else {
    debugPrint("❌ Error en lectura Modbus. Código: " + String(result));
    return -1;
  }
}


void actualizarEstado(EstadoSistema estado, const String& id, const String& descripcion) {
  estadoActual = estado;
  ultimoEstado = estado;
  String mensaje;
  switch (estado) {
    case ESTADO_OK:
      digitalWrite(LED_ROJO, LOW);
      parpadearLed(LED_VERDE, 5, false);
      falloCriticoPrevio = 0;
      break;

    case FALLO_CRITICO:
      digitalWrite(LED_VERDE, LOW);
      digitalWrite(LED_ROJO, HIGH);
      if (falloCriticoPrevio == 0) {
        String fechaHora = fechaActual(rtc.now());
        debugPrint("Fallo crítico detectado. Reiniciando el sistema.");
        mensaje = "⚠️ FALLO CRITICO\nID: " + id + "\nHora: " + fechaHora + "\n" + descripcion + "\nAcción: Reinicio";
        enviarSMS(mensaje);
        falloCriticoPrevio = 1;
        delay(1000);
        ESP.restart();
      } else {
        String fechaHora = fechaActual(rtc.now());
        debugPrint("Fallo crítico persistente. El sistema se apagará.");
        mensaje = "❌ FALLO CRITICO REPETIDO\nID: " + id + "\nHora: " + fechaHora + "\n" + descripcion + "\nAcción: Modo sleep hasta corregir.";
        enviarSMS(mensaje);
        
        digitalWrite(MODEM_PWR, LOW);
        digitalWrite(POWER_CTRL_PIN, LOW);
        delay(500);
        esp_deep_sleep_start();
      }
      break;

    case FALLO_MEDIO:
      digitalWrite(LED_VERDE, LOW);
      parpadearLed(LED_ROJO, 4, true);
      break;

    case FALLO_NO_CRITICO:
      digitalWrite(LED_VERDE, LOW);
      parpadearLed(LED_ROJO, 3, true);
      break;
  }
}

void parpadearLed(uint8_t pinLed, uint8_t cantidad, bool largo) {
  for (uint8_t i = 0; i < cantidad; i++) {
    digitalWrite(pinLed, HIGH);
    delay(largo ? 1000 : 300);
    digitalWrite(pinLed, LOW);
    delay(largo ? 500 : 200);
  }
}