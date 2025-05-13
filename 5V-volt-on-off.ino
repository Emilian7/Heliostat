const int powerControlPin = 12; // GPIO pin connected to the control pin of the Solar Power Manager

void setup() {
  // Initialize the control pin as an output
  pinMode(powerControlPin, OUTPUT);
    Serial.begin(115200);  //Data transfer speed at port
  Serial.println(powerControlPin);
}

void loop() {
  // Turn on the 5V output
  digitalWrite(powerControlPin, HIGH);
  Serial.println("turing on");
  // Wait for 5 seconds
  delay(1000);

  // Turn off the 5V output
  digitalWrite(powerControlPin, LOW);
  Serial.println("turning off");
  delay(1000);

  pinMode(A13, INPUT);
  int analogValue = analogRead(A13);
  float voltage = (analogValue / 4095.0) * 3.3 * 2;
  Serial.print(voltage);
  Serial.print(" Volts");
  Serial.println(" ");
}


