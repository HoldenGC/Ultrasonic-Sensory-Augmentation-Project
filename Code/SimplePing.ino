// SimplePing - Simple program to interface and test HC-SR04 module
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
#include <LiquidCrystal.h>

// Change pin values to appropriate pin you are using on Arduino
#define TRIG_PIN          26    // Pin # connected to Ultrasonic Trigger
#define ECHO_PIN          27    // Pin # connected to Ultrasonic Echo
#define MAX_DISTANCE_CM   274   // Maximum distance sensor can read in CM

#define MAX_WAIT_TIME     MAX_DISTANCE_CM*58

unsigned long roundtripTime;    // Microseconds until ping returns
int distance;                   // Distance calculation from microseconds

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Setup LCD object

void setup() {          // Initialization routine only runs once 
//    Start Serial communication
//    Serial.begin(115200);
    // Define pin modes
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    // Bring trigger low as part of initialization
    digitalWrite(TRIG_PIN, LOW);

    // Initialize LCD display
    lcd.begin(16, 2);              // Define 16 character x 2 line
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Booting up...."); // print a simple message
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Dist: ");
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
      
      // CODE THAT DOES SOMETHING WITH THE RESULTS GOES HERE

      // Output distance in CM to serial for testing purposes
//      Serial.print (distance);
//      Serial.println("cm");

      // Output distance in CM to LCD for testing purposes
      lcd.setCursor(6,0);  
      lcd.print("   ");
      lcd.setCursor(6,0);
      lcd.print(distance);  
        
      // Wait 1/10 of a second before beginning loop again
      delay(100);  
 
 }  // END of main loop
