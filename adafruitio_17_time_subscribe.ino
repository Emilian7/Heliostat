#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"
#include <SolarPosition.h>

// Constants
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */

// Global variables
long long deltaTime = 0;  // Initialize to 0
long long errorTime = 0;
int gotHour;
bool got_Time;
long startTime = 0;  // Initialize to 0, not false

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

int nightHour;
int nightMin;

int timeHour;
int timeMin;
int timeDay;
int timeMonth;
int timeYear;


long long currentTime;
long time2sleep = 60;  // 1 hour in seconds for example

AdafruitIO_Time* seconds = io.time(AIO_TIME_SECONDS);

void setup() {
  Serial.begin(115200);
  seconds->onMessage(handleSeconds);
  gotHour = timeHour;

  startTime = millis();



  Connect_To_Internet();
  delay(1000);



  Serial.println("Time assessment was not correct");
  Calculate_Sun_Position(timeHour, timeMin, 0, timeDay, timeMonth, timeYear);


  Serial.println("before deep sleep");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  Serial.println("Right before deep sleep");
  esp_deep_sleep_start();
  Serial.println("No deep sleep");
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
  sun_azimuth = azimuth / DEG_TO_RAD;
  sun_elevation = elevation / DEG_TO_RAD;
  sun_azimuth = map(sun_azimuth, 90, 270, 0, 180);  //maps the sun_azimuth so that 90 degreas is to the south, and 180 to the west.
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(sun_elevation, 2) + "\t\t" + String(sun_azimuth, 2));  // Map the sun_azimuth 90-270 to 0-180 so that south is 90 deeagreas
}
/*
long JulianDate(int year, int month, int day) {
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

void loop() {}

/*
#include <esp_wifi.h>
#include <WiFi.h>
#include "config.h"

// Constants
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds 

// Global variables
long long deltaTime = 0;        // Initialize to 0
long long errorTime = 0;
int gotHour;                    
bool got_Time;
long startTime = 0;             // Initialize to 0, not false

int azipulse;                   
int elepulse;                   

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;
float Lon = 13.0 * DEG_TO_RAD,
      Lat = 55.6 * DEG_TO_RAD,
      elevation,
      azimuth;

float sun_azimuth;  
float sun_elevation;  

int nightHour;      
int nightMin;       

int timeHour;   
int timeMin;    
int timeDay;    
int timeMonth;  
int timeYear;   
RTC_DATA_ATTR int fTimeHour;  
RTC_DATA_ATTR int fTimeMin;   
RTC_DATA_ATTR int fTimeDay;   
RTC_DATA_ATTR int fTimeMonth; 
RTC_DATA_ATTR int fTimeYear;  
RTC_DATA_ATTR long long futureTime; 

long long currentTime;
long time2sleep = 60;  // 1 hour in seconds for example

AdafruitIO_Time *seconds = io.time(AIO_TIME_SECONDS);   

void setup() {
  Serial.begin(115200);
  seconds->onMessage(handleSeconds);
  gotHour = timeHour;
  
  startTime = millis();
  Calculate_Sun_Position(fTimeHour, fTimeMin, 0, fTimeDay, fTimeMonth, fTimeYear);
  
  deltaTime = futureTime;
  Connect_To_Internet();
  delay(1000);
  errorTime = abs(currentTime - deltaTime);
  Serial.print("delta: ");
  Serial.println(errorTime);
  
  //if(errorTime > 300){
    Serial.println("Time assessment was not correct");
    Calculate_Sun_Position(timeHour, timeMin, 0, timeDay, timeMonth, timeYear); 
  //}
  Serial.print(deltaTime);

  Serial.flush();
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void Connect_To_Internet() {
  io.connect();
  while(io.status() < AIO_CONNECTED) {
      Serial.println(io.statusText());
      delay(1000);
  }
  Serial.println(io.statusText());
  while(!got_Time) {
    io.run();
  }
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year) {
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JD, JDx;
  int zone = 0;  //Unused variable but retained for continuity
  JD = JulianDate(year, month, day);
  JD_frac = (hour + minute / 60.0 + second / 3600.0) / 24.0 - 0.5;
  T = JD - 2451545; T = (T + JD_frac) / 36525.0;
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
  sun_azimuth = azimuth / DEG_TO_RAD;
  sun_elevation = elevation / DEG_TO_RAD;
  Serial.println("Longitude and latitude " + String(Lon / DEG_TO_RAD, 3) + " " + String(Lat / DEG_TO_RAD, 3));
  Serial.println("Year\tMonth\tDay\tHour\tMinute\tSecond\tElevation\tAzimuth");
  Serial.print(String(year) + "\t" + String(month) + "\t" + String(day) + "\t" + String(hour - zone) + "\t" + String(minute) + "\t" + String(second) + "\t");
  Serial.println(String(sun_elevation, 2) + "\t\t" + String((sun_azimuth) -180, 2)); // make a varible for the elevation and azimuth if azimuth is less or more than 180 give it a different value
}

long JulianDate(int year, int month, int day) {
  long JD;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JD = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD;
}

void handleSeconds(char *data, uint16_t len) {
  time_t seconds = strtoul(data, 0, 10);
  seconds = seconds; //seconds = seconds + 7200;
  futureTime = seconds + time2sleep;
  currentTime = seconds;
  tm* current_time = gmtime(&seconds);
  timeYear = current_time->tm_year + 1900;
  timeMonth = current_time->tm_mon + 1;
  timeDay = current_time->tm_mday;
  timeHour = current_time->tm_hour;
  timeMin = current_time->tm_min;
  tm* future_time = gmtime(&futureTime);
  fTimeYear = future_time->tm_year + 1900;
  fTimeMonth = future_time->tm_mon + 1;
  fTimeDay = future_time->tm_mday;
  fTimeHour = future_time->tm_hour;
  fTimeMin = future_time->tm_min;
  got_Time = true;
}

void loop() {}
*/
