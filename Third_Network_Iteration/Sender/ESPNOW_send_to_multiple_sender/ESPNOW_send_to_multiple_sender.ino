/***
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
***/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress1[] = {0xEC, 0xDA, 0x3B, 0xBE, 0x69, 0x70};
uint8_t broadcastAddress2[] = {0xEC, 0xDA, 0x3B, 0xBE, 0x66, 0x18};
uint8_t broadcastAddress3[] = {0x54, 0x32, 0x04, 0x88, 0xA1, 0x30};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int id; // must be unique for each sender board
  int output;
  //  int y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//colorsensor
#define NLOOP 20
#define WAIT 50
#define R D3
#define G D2
#define B D1

// Define pin numbers
const int sensorPin = A0; // Photodetector (or another sensor) connected to analog pin A2

const int stablize_size = 10;
int stb[stablize_size];

const int accer_size = 30;
int acc[accer_size];

uint8_t last_stage = 0;
uint8_t cur_val = 0;
int out_val = 90;

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  //color sensor
  analogReadResolution(8); // UNO doesn't support this
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(sensorPin, INPUT);

  digitalWrite(R, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(B, HIGH);

  for (int i = 0; i < stablize_size; i++) {
    stb[i] = 0;
  }
  for (int i = 0; i < accer_size; i++) {
    acc[i] = 0;
  }
}

void loop() {
  // Set id to send
  myData.id = 2;
  //  myData.x = random(0, 50);
  //  myData.y = random(0, 50);

  //color value -> motor valur
  //  rgb_test();
  // Serial.println(analogRead(sensorPin));
  // rgb_test();

  // read value
  cur_val = read_bar(R, G, B, sensorPin);

  // put vals in stb array
  for (uint8_t i = 1; i < stablize_size; i++) {
    stb[i - 1] = stb[i];
  }
  stb[stablize_size - 1] = cur_val;

  // get current stable stage, check if all items equal to the first item in the array
  int cur_stage = get_stable_stage(stb);
  out_val += get_increment(cur_stage, last_stage, 3);

  // put vals in stb array
  for (uint8_t i = 1; i < accer_size; i++) {
    acc[i - 1] = acc[i];
  }
  acc[accer_size - 1] = out_val;

  // clamp the value
  if (out_val > 180) out_val = 180;
  if (out_val < 0) out_val = 0;

  delay(10);
  myData.output = out_val;

  if (last_stage != cur_stage) {
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(0, (uint8_t *) &myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  //  delay(10000);

  // update last stage
  last_stage = cur_stage;
  Serial.println(myData.output);
}

int get_stable_stage(int readings[]) {
  // Serial.print("  ");
  // for (int i = 0; i < stablize_size; i++){
  //    Serial.print(readings[i]);
  // }
  // Serial.print("  ");


  for (int i = 0; i < stablize_size; i++) {
    if (readings[0] != readings[i]) {
      return readings[0];
    }
  }
  return readings[stablize_size - 1];
}


float get_accerlation(int readings[]) {
  float sum_diff = 0;
  for (int i = 0; i < accer_size - 1; i++) {
    sum_diff += float(readings[i + 1] - readings[i]);
  }
  return sum_diff / float(accer_size);
}


int get_increment(int cur_stage, int last_stage, int increment) {
  if (cur_stage == last_stage) return 0;
  else if (cur_stage - last_stage == 1 || cur_stage - last_stage == -2) return increment;
  else if (cur_stage - last_stage == -1 || cur_stage - last_stage == 2) return -increment;
  else return 0;
}

void rgb_test() {
  digitalWrite(R, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(B, HIGH);
  delay(100);

  digitalWrite(R, LOW);
  digitalWrite(G, HIGH);
  digitalWrite(B, HIGH);
  delay(100);

  digitalWrite(R, HIGH);
  digitalWrite(G, LOW);
  digitalWrite(B, HIGH);
  delay(100);

  digitalWrite(R, HIGH);
  digitalWrite(G, HIGH);
  digitalWrite(B, LOW);
  delay(100);
}

int read_bar(int PIN_R, int PIN_G, int PIN_B, int PIN_PHO) {
  // float off = 0; // Accumulated value when LED is off
  float R_on = 0;
  float G_on = 0;
  float B_on = 0;

  // Accumulate readings
  for (int count = 0; count < NLOOP; ++count) {
    // Measure with LED off
    // digitalWrite(R, HIGH);
    // digitalWrite(G, HIGH);
    // digitalWrite(B, HIGH);
    // delayMicroseconds(WAIT); // Short delay to ensure LED is off before reading
    // off += analogRead(sensorPin); // Read and accumulate sensor value

    // Measure with LED on
    digitalWrite(PIN_R, LOW);
    digitalWrite(PIN_G, HIGH);
    digitalWrite(B, HIGH);
    delayMicroseconds(WAIT); // Short delay to ensure LED is on before reading
    R_on += analogRead(PIN_PHO); // Read and accumulate sensor value

    digitalWrite(PIN_R, HIGH);
    digitalWrite(PIN_G, LOW);
    digitalWrite(PIN_B, HIGH);
    delayMicroseconds(WAIT); // Short delay to ensure LED is on before reading
    G_on += analogRead(PIN_PHO); // Read and accumulate sensor value

    digitalWrite(PIN_R, HIGH);
    digitalWrite(PIN_G, HIGH);
    digitalWrite(PIN_B, LOW);
    delayMicroseconds(WAIT); // Short delay to ensure LED is on before reading
    B_on += analogRead(PIN_PHO); // Read and accumulate sensor value
  }
  // Calculate average values
  float rr = 255 - R_on / (float)NLOOP;
  float gg = 255 - G_on / (float)NLOOP;
  float bb = 255 - B_on / (float)NLOOP;
  rr *= 0.8;

  // uint8_t r = off - R_on;
  // uint8_t g= off - G_on;;
  // uint8_t b = off - B_on;

  // Serial.print(" | rr: ");
  // Serial.print(rr);
  // // Serial.print("raw_R: ");
  // // Serial.print(R_on);
  // Serial.print(" --- ");

  // Serial.print(" | gg: ");
  // Serial.print(gg);
  // // Serial.print(" -- raw_G: ");
  // // Serial.print(G_on);
  // Serial.print(" --- ");

  // Serial.print(" | bb: ");
  // Serial.print(bb);
  // // Serial.print(" -- raw_B: ");
  // // Serial.print(B_on);
  // Serial.print(" ------ ");


  // rr: 1, gg: 2, bb: 3
  if (rr > gg && rr > bb) return 1;
  if (gg > rr && gg > bb) return 2;
  if (bb > rr && bb > gg) return 3;
  return -1;
}
