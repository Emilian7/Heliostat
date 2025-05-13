void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);

  pinMode(A13, INPUT);
}

void loop() {
  // read the analog / millivolts value for pin 2:
  int analogValue = analogRead(A13);
  //Serial.print("raw ");
  //Serial.print(analogValue);

  float voltage = (analogValue / 4095.0) * 3.3 * 2;


  // print out the values you read:
  //Serial.printf("ADC analog value = %f, %d", voltage, analogValue);  // voltage is the wanted value.
  //Serial.println(voltage);  
  Serial.println(voltage);
  Serial.print(" Volts");
  Serial.println(" ");
 
  delay(1000);  // delay in between reads for clear read from serial
}
