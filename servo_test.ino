/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_INA219.h>
// called this way, it uses the default address 0x40

//Adafruit_INA219 ina219;
Adafruit_INA219 ina219(0x41);  // Use the desired address (e.g., 0x41)
// Constants

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//Towerpro max / min 700/145
#define SERVOMIN 95   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 520   // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates has to be 50 for the used servos 

const int powerControlPin = 12; // GPIO pin connected to the control pin of the Solar Power Manager


void setup() {
  Serial.begin(115200);  //Data transfer speed at port
  Serial.println("8 channel Servo test!");

 pinMode(powerControlPin, OUTPUT);
  Serial.println(powerControlPin);
  digitalWrite(powerControlPin, HIGH);
  
while(!pwm.begin()){
  Serial.println("Sevro controller not detected");
}
 Serial.println("Sevro controller connected");   
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}


void loop() {
  
  // Drive each servo one at a time using setPWM()
 
  int angle = 90; //The angle witch the servo rotates between 0 - 180
  int pulseLenght = map(angle,0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle"); Serial.println(angle);
  Serial.print("PulseLenght"); Serial.println(pulseLenght);
  pwm.setPWM(0, 0, pulseLenght);
  pwm.setPWM(3, 0, pulseLenght);
  
  delay(10000);

  angle = 90; //The angle witch the servo rotates between 0 - 180
  pulseLenght = map(angle,0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle"); Serial.println(angle);
  Serial.print("PulseLenght"); Serial.println(pulseLenght);
  pwm.setPWM(0, 0, pulseLenght); 
  pwm.setPWM(3, 0, pulseLenght);
  delay(10000);
 

   //test to find the maximum pulselengt for witch the servo still moves
   
  //pwm.setPWM(0, 0, 95);   
  //delay(500);

  //pwm.setPWM(0, 0, 160); 
  //delay(500);
   

  
}
