#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define ESP32_RX 20
#define ESP32_TX 21

// Instantiate a new HardwareSerial object for communicating with another device
HardwareSerial MySerial(0); // UART0 on ESP32 Firebeetle

typedef struct struct_message {
  int id;       // Board ID
  int output;   // Output value (e.g., angle for stepper motor)
} struct_message;

struct_message myData;
//int outputs[2] = {0, 0};  // Array to hold output values for each board-- i think we can hold the values like this but does this mean changing the sending code?

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
//struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[2] = {board1, board2};

//calback
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
  boardsStruct[myData.id - 1].output = myData.output;
  //boardsStruct[myData.id-1].y = myData.y;
  Serial.printf("output value: %d \n", boardsStruct[myData.id - 1].output);

  String dataString = String(myData.id) + "," + String(myData.output)+'\n';
//  char outputStr[20];
//  snprintf(outputStr, sizeof(outputStr), "%d,%d\n", myData.id, "%d,%d\n", boardsStruct[myData.id - 1].output);
  Serial.println(dataString);
  MySerial.print(dataString);

  //  MySerial.println(boardsStruct[myData.id-1].output);
  //Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
  Serial.println();
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
