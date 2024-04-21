#include <AccelStepper.h>

// Define stepper motor connections and stepper object
const byte enablePin1 = 8;  // Enable pin for the first stepper motor
const byte enablePin2 = 9;  // Enable pin for the second stepper motor
AccelStepper stepper1(AccelStepper::DRIVER, 2, 5);  // Pin 2 = step, Pin 5 = direction
AccelStepper stepper2(AccelStepper::DRIVER, 3, 6);  // Pin 3 = step, Pin 6 = direction

char inputBuffer[32];  // Buffer to hold incoming data
bool newData = false;  // Flag to check if new data has been read

void setup() {
  Serial.begin(9600);  // Initialize serial port
  Serial.println("Ready to receive data...");

  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  digitalWrite(enablePin1, LOW);  // Enable the first stepper motor
  digitalWrite(enablePin2, LOW);  // Enable the second stepper motor

  // Setup the stepper motors
  stepper1.setMaxSpeed(200);
  stepper1.setAcceleration(400);
  stepper2.setMaxSpeed(5000);
  stepper2.setAcceleration(1000);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
}

void loop() {
  readSerialData();
  if (newData) {
    parseData();
    newData = false; // Reset the flag
  }

  // Continuously try to move the stepper motors to their target positions
  if (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }
  if (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }
}

void readSerialData() {
  static byte idx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0) {
    rc = Serial.read();
    if (rc != endMarker) {
      if (idx < sizeof(inputBuffer) - 1) {
        inputBuffer[idx++] = rc;
      }
    } else {
      inputBuffer[idx] = '\0'; // Terminate the string
      idx = 0;
      newData = true;
      Serial.print("Buffer: ");  // Show the complete buffer
      Serial.println(inputBuffer);
    }
  }
}

void parseData() {
  char *idStr = strtok(inputBuffer, ",");
  char *angleStr = strtok(NULL, ",");

  if (idStr != NULL && angleStr != NULL) {
    int id = atoi(idStr);
    int angle = atoi(angleStr);
    Serial.print("ID: "); Serial.println(id);
    Serial.print("Angle: "); Serial.println(angle);

    long targetPosition = map(angle, 0, 180, 0, 180);
//    long targetPosition2 = map(angle, 0, 180, 0, 720);// Map angle to steps (modify as needed)

    if (id == 3) {
      stepper1.moveTo(targetPosition);
      Serial.print("Stepper 1 moving to: "); Serial.println(targetPosition);
    } else if (id == 1) {
      stepper2.moveTo(targetPosition);
      Serial.print("Stepper 2 moving to: "); Serial.println(targetPosition);
    }
  } else {
    Serial.println("Error: Data not properly formatted.");
  }
}
