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

#include "Adafruit_SHT4x.h"
#include "config.h"
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>

#include <Adafruit_GFX.h>          // Core graphics library
#include <Fonts/FreeSerif9pt7b.h>  // Includes the selected font
#include <Adafruit_ST7789.h>       // Hardware-specific library for ST7789
#include <SPI.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;
Adafruit_MAX17048 maxlipo;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
int time_to_sleep = 600;          /* Time ESP32 will go to sleep (in seconds) */

#define PIXElPIN PIN_NEOPIXEL
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(1, PIXElPIN, NEO_GRB + NEO_KHZ800);

RTC_DATA_ATTR int total_net_attempts = 0;  //Saves the amount of times the Esp tries to connect

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);  //Makes the screen an object

/************************ Code Starts Here *******************************/

// set up the group
AdafruitIO_Group *group = io.group("Slut_projekt");

int connection_attempt = 0;
const int WakeUpPin = 2;  //Wake up pin is set to D2 as D1 does not work and always returns HIGH
static RTC_DATA_ATTR struct timeval sleep_enter_time;
struct timeval now;

float cellVoltage = 0;

void setup() {
  // start the serial connection
  Serial.begin(115200);


  if (!sht4.begin()) {                 // If the temp / hum sensor is not found it will message it on the TFT and the sleep an hour
    pinMode(TFT_BACKLITE, OUTPUT);     //Sets the blacklite to output
    digitalWrite(TFT_BACKLITE, HIGH);  //Turns on the blacklite

    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);  //Turns on I2C bus to the tft display

    tft.init(135, 240);  // Init ST7789 240x135 135 x 240 pixels
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);  //  tft.fillScreen(ST77XX_BLACK) //Makes the sceen fully black

    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(3);
    tft.println("Can't find");
    tft.println("temp sensor");
    delay(2000);

    time_to_sleep = 3600;
    Start_sleep();
  }

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
  for (samples; samples < 100; samples++) {  //loop to get the averge charging curent. Increased amount off samples takes longer to go through
    current_mA = ina219.getCurrent_mA();     // Total current divided by total readings
    total_current = (total_current + current_mA);
  }
  float average_current = (total_current / samples);
  samples= 0;

  sensors_event_t temp, humidity;

  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
  timestamp = millis() - timestamp;

  
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

  pinMode(WakeUpPin, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WakeUpPin, HIGH);  // wakes up the esp if wake up botton is pressed
  print_wakeup_reason();
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0 && wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) {  //If the reset button is pressed the ESP will read sensors, display data on the TFT and then send to AIO

    pinMode(TFT_BACKLITE, OUTPUT);     //Sets the blacklite to output
    digitalWrite(TFT_BACKLITE, HIGH);  //Turns on the blacklite

    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);  //Turns on I2C bus to the tft display

    tft.init(135, 240);  // Init ST7789 240x135 135 x 240 pixels
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);  //  tft.fillScreen(ST77XX_BLACK) //Makes the sceen fully black
    tft.setFont(&FreeSerif9pt7b);  //Sets the selsected font

    tft.setCursor(0, 15);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(0.4);
    tft.println("Emilian");

    tft.setCursor(0, 30);
    tft.setTextColor(ST77XX_YELLOW);
    tft.println("Temp: ");
    tft.setCursor(0, 50);
    tft.println("Hum: ");
    tft.setCursor(0, 70);
    tft.println("Batt_V:");
    tft.setCursor(0, 90);
    tft.print("Batt_P:");

    tft.setCursor(100, 30);
    tft.setTextColor(ST77XX_RED);

    tft.print(temp.temperature, 1);
    tft.print("C");
    tft.setCursor(100, 50);
    tft.print(humidity.relative_humidity, 0);
    tft.print("%");

    tft.setCursor(100, 70);
    tft.print(cellVoltage, 2);
    tft.print("V");
    tft.setCursor(100, 90);
    tft.print(maxlipo.cellPercent(), 1);
    tft.println("%");
    tft.print("SSID: ");
    tft.print(WIFI_SSID);
  }
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {  // Shows values on the tft when external wake up button is pressed, does not sen data. to AIO

    pinMode(TFT_BACKLITE, OUTPUT);     //Sets the blacklite to output
    digitalWrite(TFT_BACKLITE, HIGH);  //Turns on the blacklite

    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);  //Turns on I2C bus to the tft display

    tft.init(135, 240);  // Init ST7789 240x135 135 x 240 pixels
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);  //  tft.fillScreen(ST77XX_BLACK) //Makes the sceen fully black
    tft.setFont(&FreeSerif9pt7b);  //Sets the selsected font

    tft.setCursor(0, 15);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(0.4);
    tft.println("Emilian");

    tft.setCursor(0, 30);
    tft.setTextColor(ST77XX_YELLOW);
    tft.println("Temp: ");
    tft.setCursor(0, 50);
    tft.println("Hum: ");
    tft.setCursor(0, 70);
    tft.println("Batt_V:");
    tft.setCursor(0, 90);
    tft.print("Batt_P:");

    tft.setCursor(100, 30);
    tft.setTextColor(ST77XX_RED);

    tft.print(temp.temperature, 1);
    tft.print("C");
    tft.setCursor(100, 50);
    tft.print(humidity.relative_humidity, 0);
    tft.print("%");

    tft.setCursor(100, 70);
    tft.print(cellVoltage, 2);
    tft.print("V");
    tft.setCursor(100, 90);
    tft.print(maxlipo.cellPercent(), 1);
    tft.println("%");
    delay(2000);
    
  }
  io.run();

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)

  // wait for a connection
  while ( maxlipo.cellPercent()>20 ) {  //Connect to AIO, shows the current proces on the neopixel
    Serial.print(io.statusText());

    for (samples; samples < 100; samples++) {  //loop to get the averge charging curent. Increased amount off samples takes longer to go through
    current_mA = ina219.getCurrent_mA();     // Total current divided by total readings
    total_current = (total_current + current_mA);
  }
  average_current = (total_current / samples);

    cellVoltage = maxlipo.cellVoltage();
    tft.setTextColor(ST77XX_RED);
    tft.setCursor(0, 110);
    tft.fillScreen(ST77XX_BLACK);
    tft.print(io.statusText());
    tft.setCursor(100, 70);
    tft.print(cellVoltage, 2);
    tft.print("V");
    tft.setCursor(100, 90);
    tft.print(maxlipo.cellPercent(), 1);
    tft.println("%");
    tft.setCursor(100,110);
    tft.print(average_current,1);
    tft.print("mA");

    pixels.setPixelColor(1, pixels.Color(255, 0, 255));

    pixels.show();  // Send the updated pixel colors to the hardware.
    delay(60000);
    if(maxlipo.cellPercent()<21 ){
      time_to_sleep = 600;
      Start_sleep();
    }
    
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
  tft.fillRect(0, 100, 200, 240, ST77XX_BLACK);
  tft.setCursor(0, 110);
  tft.print(io.statusText());
  connection_attempt = 0;
  total_net_attempts = 0;
  Serial.print(io.statusText());
  pixels.clear();  // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();  // Send the updated pixel colors to the hardware.
  delay(200);

  if (maxlipo.cellPercent() < 3) {  //Adjust the sleeping time depening on the battery percentage
    time_to_sleep = 1200;
  }
  if (maxlipo.cellPercent() > 3 && maxlipo.cellPercent() < 6) {
    time_to_sleep = 900;
  }
  if (maxlipo.cellPercent() > 6) {
    time_to_sleep = 600;
  }

  group->set("temp", temp.temperature);
  group->set("humid", humidity.relative_humidity);
  group->set("battPerc", maxlipo.cellPercent());
  group->set("battVolt", cellVoltage);
  group->set("TimeToSleep", time_to_sleep);
  group->set("chargeCurrent", average_current);
  group->save();

  pixels.clear();  // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();  // Send the updated pixel colors to the hardware.
  delay(50);

  Serial.print("temp: ");
  Serial.println(temp.temperature);
  Serial.print("humidity: ");
  Serial.println(humidity.relative_humidity);

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

  Start_sleep();
}

void Start_sleep() {  //Send the ESP into deep sleep
  gettimeofday(&now, NULL);
  int slept_time_s = now.tv_sec - sleep_enter_time.tv_sec;
  if (slept_time_s < time_to_sleep) {
    time_to_sleep = time_to_sleep - slept_time_s;
    Serial.println(slept_time_s);
    Serial.println(time_to_sleep);
  }
  else {
  gettimeofday(&sleep_enter_time, NULL);
  }
  Serial.flush();
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void print_wakeup_reason() {  //Prints what the reason for the ESP to wake up
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);

      break;
  }
}

void loop() {}  //Loop needed to verify code, does nothing