#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define ESP32_RX 20
#define ESP32_TX 21

// Instantiate a new HardwareSerial object for communicating with another device
HardwareSerial MySerial(0); // UART0 on ESP32 Firebeetle

typedef struct struct_message {
  int id;           // Board ID
  int increment;    // Increment value (change in angle)
} struct_message;

struct_message myData;

typedef struct struct_board {
  int angle;        // Fixed angle
  int movement;     // Calculated movement (angle + increment)
} struct_board;

// Create a structure to hold the readings and movement for each board
struct_board board1 = {90, 90};  // Initialize angle to a fixed number, e.g., 90
struct_board board2 = {90, 90};  // Initialize angle to a fixed number, e.g., 90

// Create an array with all the board structures
struct_board boardsStruct[2] = {board1, board2};

// Callback
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Packet received from: ");
  Serial.println(macStr);

  // Deserialize the incoming data
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);

  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].movement = boardsStruct[myData.id - 1].angle + myData.increment;
  Serial.printf("Movement value: %d \n", boardsStruct[myData.id - 1].movement);

  String dataString = String(myData.id) + "," + String(boardsStruct[myData.id - 1].movement) + '\n';
  Serial.println(dataString);
  MySerial.print(dataString);
}

void setup() {
  Serial.begin(115200);
  MySerial.begin(9600, SERIAL_8N1, ESP32_RX, ESP32_TX);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  delay(1000);
}
