#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define ESP32_RX 20
#define ESP32_TX 21

// Instantiate a new HardwareSerial object (we are using UART0 here)
HardwareSerial MySerial(0); // For ESP32 Firebeetle, the default Serial might be Serial0

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int output;
  //  int y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
//struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[2] = {board1, board2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].output = myData.output;
  //Serial.printf("output value: %d \n", boardsStruct);
  Serial.printf(boardsStruct[2]);
  MySerial.print(boardsStruct[0].output);
  MySerial.print(",");\
  MySerial.println(boardsStruct[1].output);

  //Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
  Serial.println();
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  // Initialize the hardware serial port for the ESP32
  MySerial.begin(9600, SERIAL_8N1, ESP32_RX, ESP32_TX);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  //  MySerial.println("???");
  // Acess the variables for each board
  /*int board1X = boardsStruct[0].x;
    int board1Y = boardsStruct[0].y;
    int board2X = boardsStruct[1].x;
    int board2Y = boardsStruct[1].y;
    int board3X = boardsStruct[2].x;
    int board3Y = boardsStruct[2].y;*/

  if (MySerial.println("Hello from ESP32!") > 0) {
    // Message was sent successfully
  } else {
    // Handle the error, maybe try again or report back
    // For debugging purposes, you can uncomment the next line
    // Serial.println("Failed to send message");
  }



  delay(1000);
}
