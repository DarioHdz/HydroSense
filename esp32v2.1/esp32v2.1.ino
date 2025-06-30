/*
PIN LIST:
  + No usar:
    - D0
    - D1
    - D2
    - D3
    - D12

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
*/

#include <ArduinoHttpClient.h>
#include <Adafruit_ADS1X15.h>
#define TINY_GSM_MODEM_SIM800
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <SHT1x-ESP.h>
#include <esp_sleep.h>
#include <RTClib.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


// === DS3231 RTC ===
#define SDA_PIN        15
#define SCL_PIN        4
RTC_DS3231 rtc;

// === SHT-10 Sensor ===
#define SHT_DATA_PIN   16
#define SHT_CLOCK_PIN  17
SHT1x sht1x(SHT_DATA_PIN, SHT_CLOCK_PIN, SHT1x::Voltage::DC_3_3v);

// === Micro SD ===
#define SD_CS_PIN      5

// === ADC Sensores Suelo (ADS1115) ===
// SDA_PIN, SCL_PIN compartidos con RTC
Adafruit_ADS1115 ads;

// === RS485 Modbus ===
#define RS485_RX_PIN   33
#define RS485_TX_PIN   32
#define RS485_DE_RE    27
ModbusMaster node;

// === SIM800L Modem ===
#define MODEM_RX_PIN   26
#define MODEM_TX_PIN   25
HardwareSerial sim800(1);
TinyGsm modem(sim800);
TinyGsmClient gsmClient(modem);
const char phoneNumber[] = "+524481223450";

// === MOSFET de control ===
#define POWER_CTRL_PIN 21  // Control alimentación modulos 5v
#define MODEM_PWR_PIN  22  // Modulo SIM

// === LEDs de Estado ===
#define LED_ROJO_PIN   14
#define LED_VERDE_PIN  13

// PROGMEM: Días de la semana
const char diasSemana[][16] PROGMEM = {
  "Domingo","Lunes","Martes","Miércoles",
  "Jueves","Viernes","Sábado"
};

// Variables de estado
RTC_NOINIT_ATTR int falloCriticoPrevio = 0;
enum EstadoSistema { ESTADO_OK, FALLO_CRITICO, FALLO_MEDIO, FALLO_NO_CRITICO };

// Prototipos
void fechaActual(const DateTime& dt, char* buf, size_t len);
bool enviarSMS(const char* msg);
void guardarEnSD(const char* linea);
void actualizarEstado(EstadoSistema estado, const char* id = NULL, const char* descripcion = NULL);
void parpadearLed(uint8_t pin, uint8_t veces, bool largo);
void setupRS485Modbus();
bool verificarModbus();
bool sincronizarHoraDesdeInternet();
void mostrarNivelDeSenal();
bool conectarGPRS();
int leerRadiacionModbus();

void setup() {
  delay(5000);
  // LEDs inicialización
  pinMode(LED_ROJO_PIN, OUTPUT);
  pinMode(LED_VERDE_PIN, OUTPUT);
  digitalWrite(LED_ROJO_PIN, LOW);
  digitalWrite(LED_VERDE_PIN, LOW);

  // RTC (DS3231)
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Serial.begin(115200);
  Serial.println(F("Iniciando sistema ..."));

  // SHT-10 Sensor
  Serial.println(F("Iniciando sensor SHT-10 ..."));

  // Micro SD & ADC Suelo
  Serial.println(F("Encendiendo SD y ADC suelo ..."));
  pinMode(POWER_CTRL_PIN, OUTPUT);
  digitalWrite(POWER_CTRL_PIN, HIGH);
  delay(200);
  if (!SD.begin(SD_CS_PIN)) {
    actualizarEstado(FALLO_CRITICO, "SD", "Error al iniciar SD");
  } else {
    parpadearLed(LED_VERDE_PIN, 3, true);
    if (!SD.exists("/log.csv")) {
      File f = SD.open("/log.csv", FILE_WRITE);
      if (f) {
        f.println(F("Fecha,Temp (C),Humedad,Suelo A0,Suelo A1,Radiacion"));
        f.close();
      }
    }
  }
  if (!ads.begin()) {
    actualizarEstado(FALLO_CRITICO, "ADC", "ADS1115 no detectado");
  }

  // RS485 Modbus
  Serial.println(F("Iniciando RS485 Modbus ..."));
  setupRS485Modbus();
  if (!verificarModbus()) {
    actualizarEstado(FALLO_CRITICO, "MODBUS", "Fallo en RS485");
  }

  // SIM800L
  Serial.println(F("Encendiendo SIM800L ..."));
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, HIGH);
  delay(2000);
  sim800.begin(9600, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  modem.restart();
  if (modem.waitForNetwork(20000)) {
    parpadearLed(LED_VERDE_PIN, 3, true);
    mostrarNivelDeSenal();
  } else {
    actualizarEstado(FALLO_NO_CRITICO);
  }

  // RTC verificación
  Serial.println(F("Verificando RTC ..."));
  if (!rtc.begin()) {
    actualizarEstado(FALLO_CRITICO, "RTC", "RTC no detectado");
  } else if (rtc.lostPower()) {
    if (!sincronizarHoraDesdeInternet()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }
}

void loop() {
  DateTime now = rtc.now();

  // SHT-10 lectura
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  if (isnan(tempC) || isnan(humidity)) actualizarEstado(FALLO_MEDIO);

  // ADC suelo lectura
  int suelo0 = ads.readADC_SingleEnded(0);
  int suelo1 = ads.readADC_SingleEnded(1);

  // Radiación Modbus lectura
  int radiacion = leerRadiacionModbus();

  // Formatear CSV
  char fechaBuf[32], radStr[8], csv[128];
  fechaActual(now, fechaBuf, sizeof(fechaBuf));
  if (radiacion >= 0) snprintf(radStr, sizeof(radStr), "%d", radiacion);
  else strcpy(radStr, "NA");
  snprintf(csv, sizeof(csv), "%s,%.1f,%.1f,%d,%d,%s",
           fechaBuf, tempC, humidity, suelo0, suelo1, radStr);
  Serial.println(csv);
  guardarEnSD(csv);

  delay(20000);
}

void fechaActual(const DateTime& dt, char* buf, size_t len) {
  char dia[16];
  strcpy_P(dia, (char*)pgm_read_word(&(diasSemana[dt.dayOfTheWeek()])));
  snprintf(buf, len,
           "%04u/%02u/%02u (%s) %02u:%02u:%02u",
           dt.year(), dt.month(), dt.day(), dia,
           dt.hour(), dt.minute(), dt.second());
}

//  Enviar SMS con retiros de String
bool enviarSMS(const char* msg) {
  char buf[160];
  strncpy(buf, msg, sizeof(buf)-1);
  buf[sizeof(buf)-1] = '