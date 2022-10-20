#include <stdio.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#define uS_TO_S_FACTOR 1000ULL  /* Conversion factor for micro seconds to seconds */
int TIME_TO_SLEEP  = 60000;
int startTime = 0;
int elapsedTime = 0;
/* Time ESP32 will go to sleep (in seconds) */

int Enable_pin = 4; //to use for max485 module
Adafruit_BME280 bme; // I2C
unsigned long delayTime;

void setup() {
  //  startTime = millis();
  pinMode(18, OUTPUT); //to wake up the outside esp
  digitalWrite(18, HIGH); //for outside esp to wake up
  delay(3000); //waiting for outside esp to wake up
  Serial.begin(115200);
  //  Serial.println(startTime);
  pinMode(Enable_pin, OUTPUT); //for max485
  digitalWrite(Enable_pin, HIGH);        //  (HIGH to send value from Slave using UART)
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop() {
  // put your main code here, to run repeatedly:

  DynamicJsonDocument doc(1024); //saving data in json

  bool status = false;
  for (int i = 0; i < 5; i++) {
    status = bme.begin(0x76);
    if (!status) {
      Serial.println("Attempt " + String(i + 1) + " : Could not find a valid BME280 sensor, check wiring!");

    } else {
      break;
    }
  }

  if (status) {
    delayTime = 41;//by calculation as per sampling coeffs.
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );

    for (int i = 0; i < 5; i++) {
      doc["t"] = bme.readTemperature();
      doc["p"] = bme.readPressure() / 100.0F;
      doc["h"] = bme.readHumidity();
      if (doc["t"] == "null" || doc["t"] == "null" || doc["t"] == "null") {
        Serial.println("Attempt " + String(i + 1) + " : Values read are NULL!");
      } else {
        serializeJson(doc, Serial);//sending data over uart.
        break;
      }
    }
  }

  elapsedTime = millis();
  esp_sleep_enable_timer_wakeup((TIME_TO_SLEEP - elapsedTime) * uS_TO_S_FACTOR);
  Serial.println("\nTime to wake up: " + String(TIME_TO_SLEEP - elapsedTime));
  esp_deep_sleep_start();
}
