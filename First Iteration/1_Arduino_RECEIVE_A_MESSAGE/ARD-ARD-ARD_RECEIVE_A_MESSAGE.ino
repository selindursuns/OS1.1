void setup() {
  // Start the built-in serial port, connected to the computer.
  Serial.begin(9600);
}

void loop() {
  // Check if data is available to read.
  if (Serial.available() > 0) {
    // Read the incoming message until a newline is received
    String message = Serial.readStringUntil('\n');
    
    // Display the message in the Serial Monitor of the Arduino IDE
    
    Serial.print("Message received: ");
    Serial.println(message);
  }
}
