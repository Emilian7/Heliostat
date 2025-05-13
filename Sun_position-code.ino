#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SolarPosition.h>
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;

// Constants
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
int time_to_sleep = 20;           /* Time ESP32 will go to sleep (in seconds) */

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 95    // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 520   // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates


AdafruitIO_Group* group = io.group("Heliostat");

// Global variables

int gotHour;
bool got_Time;

int azipulse;
int elepulse;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
float Lon = 13.0 * DEG_TO_RAD,  // Malmös kordinates
  Lat = 55.6 * DEG_TO_RAD,
      elevation,
      azimuth;

float sun_azimuth;
float sun_elevation;

int timeHour;
int timeMin;
int timeDay;
int timeMonth;
int timeYear;

RTC_DATA_ATTR int total_net_attempts = 0;  //Saves the amount of times the Esp tries to connect
int connection_attempt = 0;


long long currentTime;


AdafruitIO_Time* seconds = io.time(AIO_TIME_SECONDS);

void setup() {
  Serial.begin(115200);

  delay(1000);  //Take some time to open up the Serial Monitor
  
  print_wakeup_reason();

  if (!ina219.begin()) {  // if the charge reader isn't detected
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_16V_400mA();
  float current_mA = 0;
  float total_current = 0;
  float samples = 0;
  for (samples; samples < 500; samples++) {  //loop to get the averge charging curent. Increased amount off samples takes longer to go through
    current_mA = ina219.getCurrent_mA();     // Total current divided by total readings
    total_current = (total_current + current_mA);
  }
  float average_current = (total_current / samples);

  analogReadResolution(12);

  

  pinMode(A13, INPUT);
  int analogValue = analogRead(A13);
  float voltage = (analogValue / 4095.0) * 3.3 * 2;
  Serial.print(voltage);
  Serial.print(" Volts");
  Serial.println(" ");

  if (voltage < 3.5) {
    time_to_sleep = 1800;
    Serial.println("low voltage, going to sleep");
    Start_sleep();
  }


  seconds->onMessage(handleSeconds);
  gotHour = timeHour;

  Connect_To_Internet();
  delay(1000);

  Calculate_Sun_Position(timeHour, timeMin, 0, timeDay, timeMonth, timeYear);


  if (sun_elevation < -40) {
    time_to_sleep = 14400;  //sleep for 4 hours
  } else if (sun_elevation < -20) {
    time_to_sleep = 7200;
  } else if (sun_elevation < -10) {
    time_to_sleep = 3600;
  } else if (sun_elevation < -5) {
    time_to_sleep = 1800;
  } else {
    //time_to_sleep = 600;
  }

  if (sun_elevation > 0) {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    int elevation_pulseLenght = map(sun_elevation, 0, 180, SERVOMIN, SERVOMAX);
    int azimuth_pulseLenght = map(sun_azimuth, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Elevation pulse lenght");
    Serial.println(elevation_pulseLenght);
    Serial.print("Azimuth pulse lenght");
    Serial.println(azimuth_pulseLenght);
    pwm.setPWM(0, 0, azimuth_pulseLenght);    //zero horizontel
    pwm.setPWM(1, 0, elevation_pulseLenght);  //one vertikal
    delay(500);
  }
  group->set("HchargeCurrent", average_current);
  group->set("Volts", voltage);
  group->set("SleepTime",time_to_sleep);
  group->save();

  Serial.print("Sleeping time");
  Serial.print(time_to_sleep);
  Start_sleep();
}


void Connect_To_Internet() {
  io.connect();
  while (io.status() < AIO_CONNECTED) {
    Serial.println(io.statusText());
    delay(1000);
  }
  Serial.println(io.statusText());
  while (!got_Time) {
    io.run();
  }
}
void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JD, JDx;
  int zone = 0;  //Unused variable but retained for continuity
  JD = JulianDate(year, month, day);
  JD_frac = (hour + minute / 60.0 + second / 3600.0) / 24.0 - 0.5;
  T = JD - 2451545;
  T = (T + JD_frac) / 36525.0;
  L0 = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
  M = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e = 0.016708617 - 0.000042037 * T;
  C = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f = M + C;
  Obl = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx = JD - 2451545;
  GrHrAngle = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle = fmod(GrHrAngle, 360.0);
  L_true = fmod(C + L0, 2 * PI);
  R = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  RA = atan2(sin(L_true) * cos(Obl), cos(L_true));
  Decl = asin(sin(Obl) * sin(L_true));
  HrAngle = DEG_TO_RAD * GrHrAngle + Lon - RA;
  elevation = asin(sin(Lat) * sin(Decl) + cos(Lat) * (cos(Decl) * cos(HrAngle)));
  azimuth = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(Lat) - tan(Decl) * cos(Lat));
  sun_azimuth = (azimuth / DEG_TO_RAD), 2;
  sun_elevation = elevation / DEG_TO_RAD;
  sun_azimuth = map(sun_azimuth, 90, 270, 0, 180);  //maps the sun_azimuth so that 90 degreas is to the south, and 180 to the west.
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(sun_elevation, 2) + "\t\t" + String(sun_azimuth, 2));  // Map the sun_azimuth 90-270 to 0-180 so that south is 90 deeagreas
}
/*
long JulianDate(int year, int month, int day) { // Is included with <SolarPosition.h> libery
  long JD;
  int A, B;
  if (month <= 2) {
    year--;
    month += 12;
  }
  A = year / 100;
  B = 2 - A + A / 4;
  JD = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD;
}
*/
void handleSeconds(char* data, uint16_t len) {
  time_t seconds = strtoul(data, 0, 10);
  seconds = seconds;  //seconds = seconds + 7200;
  currentTime = seconds;
  tm* current_time = gmtime(&seconds);
  timeYear = current_time->tm_year + 1900;
  timeMonth = current_time->tm_mon + 1;
  timeDay = current_time->tm_mday;
  timeHour = current_time->tm_hour;
  timeMin = current_time->tm_min;
  got_Time = true;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void Start_sleep() {  //Send the ESP into deep sleep
  Serial.flush();
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void loop() {}
