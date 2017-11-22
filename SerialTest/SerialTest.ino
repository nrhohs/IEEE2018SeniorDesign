void setup() {
  Serial.begin(57600); // opens serial port, sets data rate to 57600 baud
}
 
void loop() {
  while (Serial.available() > 0) { // if any data available
    char incomingByte = Serial.read(); // read byte
    Serial.write(incomingByte); // send it back
  }
}

