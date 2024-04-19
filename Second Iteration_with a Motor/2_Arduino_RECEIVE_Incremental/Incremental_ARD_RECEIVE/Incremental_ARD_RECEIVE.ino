#include <AccelStepper.h>

const byte enablePin = 8;
AccelStepper stepper(AccelStepper::DRIVER, 2, 5); 
//AccelStepper stepper(AccelStepper::DRIVER, 3, 6); 
//AccelStepper stepper(AccelStepper::DRIVER, 4, 7); 

void setup() {
    Serial.begin(9600); 
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW); 

    stepper.setAcceleration(1000); 
    stepper.setMaxSpeed(1000);     
    stepper.setCurrentPosition(0); 
}

void loop() {
    if (Serial.available() > 0) {
        // Read the incoming message until a newline is received
        String message = Serial.readStringUntil('\n');
        int angle = message.toInt(); 

        // Map the angle directly to a step position
        long targetPosition = map(angle, 0, 180, 0, 180); 

        stepper.moveTo(targetPosition); // Set the target position
        Serial.print("Angle received: ");
        Serial.print(angle);
        Serial.print(", Moving to step: ");
        Serial.println(targetPosition);
    }

    // Continuously try to move the stepper to the target position
    if (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}
//add another motor code here for 
