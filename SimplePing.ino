// MyPing - Simple program to interface and test HC-SR04 module
// by G. C. Holden

// Program to allow an Arduino to utilize an HC-SR04 
// Ultrasonic sensor. The wiring is as follows:
// Arduino Pin    to    Sensor Pin
// Ground               Ground
// 5v                   VCC
// 26                   Trigger
// 27                   Echo
//
// Arduino pins for trigger and echo can be changed by changing 
// the value below in the #define statement. HC-SR04 is 5v - DO
// NOT USE it with 3.3v Arduinos.

// Must include the Arduino Library
#include <Arduino.h>

// Change pin values to appropriate pin you are using on Arduino
#define TRIG_PIN          26    // Pin # connected to Ultrasonic Trigger
#define ECHO_PIN          27    // Pin # connected to Ultrasonic Echo
#define MAX_DISTANCE_CM   274   // Maximum distance sensor can read in CM


void setup() {          // Initialization routine only runs once 
    // Start Serial communication
    Serial.begin(115200);
    // Define pin modes
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    // Bring trigger low as part of initialization
    digitalWrite(TRIG_PIN, LOW);
} // END of initialization 


void loop() {          // Main loop runs for the rest of time

      // Basic sequence to signal HC-SR04 to Transmit
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(4);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(11);  
      digitalWrite(TRIG_PIN, LOW);

      // Call Arduino pulseIn() to have HC-SR04 listen for ping
      roundtripTime = pulseIn(ECHO_PIN, HIGH, MAX_WAIT_TIME);

      // Calculate one way distance in CM - Change 58 to 148 for inches
      distance = roundtripTime / 58;
      
      // Output distance in CM for testing purposes
      Serial.println(roundtripTime/58);

      // PLACE CODE OR FUNCTION CALL HERE TO DO SOMETHING BASED ON PING VALUE

      // Wait 1/10 of a second before beginning loop again
      delay(100);  
 
 }  // END of main loop
