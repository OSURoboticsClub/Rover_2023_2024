#include <Arduino.h>
#include <ESP32Servo.h>

/*

//Servo Motor Code
Servo myServo;
int servoPin = 22;


void setup() {
    // Necessary for ESP32 to manage PWM timers properly
    ESP32PWM::allocateTimer(0);
    
    myServo.setPeriodHertz(50);           // Standard 50Hz frequency
    myServo.attach(servoPin, 500, 2400);  // Pin 22, min pulse, max pulse
}

void loop() {
    myServo.write(90);  // Move to center (90 degrees)
    delay(1000);
    myServo.write(0);   // Move to 0 degrees
    delay(1000);
}
*/


//ESC motor Code
const int escPin = 22;
Servo myESC;

void setup() {
    Serial.begin(115200);

    // Prepare PWM Timers
    ESP32PWM::allocateTimer(0);
    myESC.setPeriodHertz(50); // ESCs MUST have 50Hz

    // Attach the ESC
    // Most ESCs use 1000us for STOP and 2000us for FULL THROTTLE
    myESC.attach(escPin, 1000, 2000);

    // --- ESC ARMING SEQUENCE ---
    Serial.println("Arming ESC... Sending Stop Signal");
    myESC.write(0);  // Send "Zero" throttle
    delay(5000);     // Wait 5 seconds for the ESC to beep and arm
    Serial.println("ESC should be armed now.");
}

void loop() {
    // Slowly speed up
    Serial.println("Increasing Throttle...");
    for (int speed = 0; speed <= 15; speed++) { // Stay low (30%) for safety
        myESC.write(speed);
        delay(100);
    }

    delay(10000);

    // Shut down
    Serial.println("Stopping...");
    myESC.write(0);
    delay(5000);
}