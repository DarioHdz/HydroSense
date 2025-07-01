#include <Arduino.h>
#include <SHT1x-ESP.h>
#include <RTClib.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// Clock
// SDA - 21
// SCL - 22
RTC_DS3231 rtc;

// SHT10
#define dataPin  15
#define clockPin 4
SHT1x sht1x(dataPin, clockPin);
float temp_c;
float temp_f;
float humidity;

bool setupSD();

void setup()
{
  delay(5000);
  Serial.begin(9600); 
  Serial.println("Starting up");

  // Clock setup
  if (! rtc.begin()) {
    Serial.println("RTC module is NOT found");
    while (1);
  }else{
    Serial.println("RTC detected setting datetime");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if(!setupSD()){
    Serial.println("SD card error!");
  }else{
    Serial.println("SD card ready!");
  }
  
}

void loop()
{
  
  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();

  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(temp_c, DEC);
  Serial.print("C / ");
  Serial.print(temp_f, DEC);
  Serial.print("F. Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  DateTime now = rtc.now();
  Serial.print("ESP32 RTC Date Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(now.dayOfTheWeek());
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  delay(2000);
}

bool setupSD(){
  #ifdef REASSIGN_PINS
    SPI.begin(sck, miso, mosi, cs);
    if (!SD.begin(cs)) {
  #else
    if (!SD.begin()) {
  #endif
      Serial.println("Card Mount Failed");
      return false;
    }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  return true;
}