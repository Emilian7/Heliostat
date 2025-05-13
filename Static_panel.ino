// Adafruit IO Group Publish Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.

#include "config.h"
#include "Adafruit_MAX1704X.h"

#include <Adafruit_GFX.h>          // Core graphics library
#include <Adafruit_ST7789.h>       // Hardware-specific library for ST7789
#include <SPI.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
Adafruit_MAX17048 maxlipo;


int connection_attempt = 0;
RTC_DATA_ATTR int total_net_attempts = 0;  //Saves the amount of times the Esp tries to connect


#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
int time_to_sleep = 600;          /* Time ESP32 will go to sleep (in seconds) */

/************************ Code Starts Here *******************************/

// set up the group
AdafruitIO_Group *group = io.group("your-group-name");


void setup() {
  // start the serial connection
  Serial.begin(115200);

  if (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_16V_400mA();  // set to a lower voltage and a more precise reading
  float current_mA = 0;
  float total_current = 0;
  float samples = 0;
  for (samples; samples < 500; samples++) {  //loop to get the averge charging curent. Increased amount off samples takes longer to go through
    current_mA = ina219.getCurrent_mA();     // Total current divided by total readings
    total_current = (total_current + current_mA);
  }
  float average_current = (total_current / samples);

  float cellVoltage = 0;
  int VoltageReadTries = 0;
  while (cellVoltage == 0 && VoltageReadTries < 5) {  //Loop until cellvoltage has a value
    cellVoltage = maxlipo.cellVoltage();
    if (isnan(cellVoltage)) {
      Serial.println("Failed to read cell voltage, check battery is connected!");
      delay(2000);
      return;
    }
    VoltageReadTries++;
  }
  VoltageReadTries = 0;

  io.run();
  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();


  // wait for a connection
  while (io.status() < AIO_CONNECTED && connection_attempt < 10) {  //Connect to AIO, shows the current proces on the neopixel
    Serial.print(io.statusText());
    
    delay(500);
    connection_attempt++;
    if (connection_attempt > 9) {  // if the Esp cant connect it will go back to sleep
      delay(50);
      total_net_attempts++;
      time_to_sleep = 60;
      Start_sleep();
    }
    if (total_net_attempts > 10) {
      time_to_sleep = 600;
      Start_sleep();
    }
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  delay(200);

  group->set("battPerc", maxlipo.cellPercent());
  group->set("battVolt", cellVoltage);
  group->set("TimeToSleep", time_to_sleep);
  group->set("chargeCurrent", average_current);
  group->save();

  Serial.print(F("Batt Voltage: "));
  Serial.print(cellVoltage, 2);
  Serial.println(" V");
  Serial.print(F("Batt Percent: "));
  Serial.print(maxlipo.cellPercent(), 1);
  Serial.println(" %");
  Serial.println("Current:       ");
  Serial.print(average_current);
  Serial.println(" mA");
  Serial.println();

  delay(2500);
  Start_sleep();
}

void Start_sleep() {  //Send the ESP into deep sleep
  Serial.flush();
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}


void loop() {}  //Loop needed to verify code, does nothing