#include <HardwareSerial.h>

//// Define the Rx and Tx pins for the ESP32 Firebeetle
//#define ESP32_RX 3  // GPIO 3 is typically used as RX0 on ESP32
//#define ESP32_TX 1  // GPIO 1 is typically used as TX0 on ESP32

#define ESP32_RX 20
#define ESP32_TX 21


// Instantiate a new HardwareSerial object (we are using UART0 here)
HardwareSerial MySerial(0); // For ESP32 Firebeetle, the default Serial might be Serial0

void setup() {
  // Initialize the hardware serial port for the ESP32
  MySerial.begin(9600, SERIAL_8N1, ESP32_RX, ESP32_TX);
}

void loop() {
  // Send a message every second
  if(MySerial.println("Hello from ESP32!") > 0) {
    // Message was sent successfully
  } else {
    // Handle the error, maybe try again or report back
    // For debugging purposes, you can uncomment the next line
    // Serial.println("Failed to send message");
  }
  delay(1000);
}
