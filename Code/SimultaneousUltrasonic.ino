// Simultaneous Ultrasonic Readings
// by G. C. Holden
// 11/22/2018
// Allows for improved consistency in ultrasonic reading by utilizing
// two inexpensive HC-SR04 modules pointed in the same direction and
// manually pulsing them nearly simultaneously, using timestamps (no interrupt timers)
// to calculate distances.


#include <Arduino.h>    // Must include the Arduino Library 
//#include <LiquidCrystal.h>  // Uncomment if using 16 x 2 LCD display for testing

#define TRIG_PIN1          8    // Pin # connected to Ultrasonic Module 1 Trigger
#define ECHO_PIN1          9    // Pin # connected to Ultrasonic Module 1  Echo

#define TRIG_PIN2          10   // Pin # connected to Ultrasonic Module 2 Trigger
#define ECHO_PIN2          11   // Pin # connected to Ultrasonic Module 2 Echo

#define MOTOR_PWM_PIN      6    // PWM Pin # connected to Motor control

#define ELEMENTS           4    // Number of elements to average together for distance reading

#define MAX_ECHO_WAIT   15892   // Maximum time to wait for echo pins to return low (distance of 274 cm * 58 microseconds seconds per cm) 
#define MAX_READY_WAIT  550     // Maximum number of microseconds to wait for echo pins to be high (ready)

// LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Uncomment to Setup LCD display object

uint8_t port1;      // Register corresponding to echo pin of ultrasonic module 1
uint8_t bitMask1;   // Bitmask corresponding to echo pin of ultrasonic module 1
uint8_t port2;      // Register corresponding to echo pin of ultrasonic module 2
uint8_t bitMask2;   // Bitmask corresponding to echo pin of ultrasonic module 2

bool echoHigh_1;    // Flag indicating that Module 1 Echo pin goes high and timestamp set
bool echoLow_1;     // Flag indicating that Module 1 Echo pin goes back low and timestamp set
bool echoHigh_2;    // Flag indicating that Module 2 Echo pin goes high and timestamp set
bool echoLow_2;     // Flag indicating that Module 2 Echo pin goes back low and timestamp set

uint32_t Tm_outOfSetLoop; // Timestamp of exiting the loop to wait for echo pins to be ready
uint32_t Tm_outOfRtnLoop; // Timestamp of exiting the loop to wait for return of ping
uint32_t Tm_pulseSent;    // Timestamp of pulse initialization routine beginning

uint32_t Tm_start1;  // Timestamp of Module 1 Echo pin going high
uint32_t Tm_end1;    // Timestamp of Module 1 Echo pin going low

uint32_t Tm_start2; // Timestamp of Module 2 Echo pin going high
uint32_t Tm_end2;   // Timestamp of Module 2 Echo pin going low

uint32_t Tm_1 = millis();      // Timestamp of beginning of one averaging cycle
uint32_t Tm_2 = millis();      // Timestamp of end of one averaging cycle

uint32_t Tm_roundtrip1;    // Microseconds until ping returns
uint32_t Tm_roundtrip2;    // Microseconds until ping returns

volatile uint8_t* addrInputRegPort1;  // Address of the register holding the input state for port 1
volatile uint8_t* addrInputRegPort2;  // Address of the register holding the input state for port 2
// Declared volatile in case ultrasonic module changes the state while temporarily stored

int dist1 = 0;                    // Temporary storage of calculated distance 1
int dist2 = 0;                    // Temporary storage of calculated distance 2
int distances[ELEMENTS + 2] = {0}; // Array holding validated distance calculations
int index = 0;                    // Array indexing
int sum = 0;                      // Accumulator for averaging distances
int average = 0;                  // Average of validated distances
int intensity = 0;                // Motor intensity


void setup() {          // Initialization routine only runs once
  // Define pin modes
  pinMode(TRIG_PIN1, OUTPUT);       // Pin used to trigger module 1
  pinMode(ECHO_PIN1, INPUT);        // Pin used to read time of flight for module 1
  pinMode(TRIG_PIN2, OUTPUT);       // Pin used to trigger module 2
  pinMode(ECHO_PIN2, INPUT);        // Pin used to read time of flight for module 2

  pinMode(MOTOR_PWM_PIN, OUTPUT);   // PWM pin used to control intensity of haptic motor

  // Store the register corresponding to Echo pin 1
  port1 = digitalPinToPort(ECHO_PIN1);
  // Store value of the current numbered pins register bitmask in pinToBitMask
  bitMask1 = digitalPinToBitMask(ECHO_PIN1);
  // Store the address of the input register of port 1
  addrInputRegPort1 = portInputRegister(port1);

  // Store the register corresponding to Echo pin 2
  port2 = digitalPinToPort(ECHO_PIN2);
  // Store value of the current numbered pins register bitmask in pinToBitMask
  bitMask2 = digitalPinToBitMask(ECHO_PIN2);
  // Store the address of the input register of port 2
  addrInputRegPort2 = portInputRegister(port2);

  //    // Uncomment block to ready LCD display
  //    // USE LCD DISPLAY FOR TESTING OF VALUES
  //    // Initialize LCD display
  //    lcd.begin(16, 2);              // Define 16 character x 2 line
  //    lcd.clear();
  //    lcd.setCursor(0,0);
  //    lcd.print("Booting up...."); // print a simple message
  //    delay(1000);
  //    lcd.clear();
  //    lcd.setCursor(0,0);
  //    lcd.print("Dist 1:     cm");
  //    lcd.setCursor(0,1);
  //    lcd.print("Elapse:     millis");

  //    //**** Uncomment block in order to output CSV Header to serial monitor ****
  //    // Start Serial communication
  //    Serial.begin(115200);
  //    // Wait 1 second to enable activation of Serial Monitor
  //    delay(1000);
  //    Serial.println( "Intensity,   port1 , bitMask1,  port2,  bitMask2, *addrInputRegPort1, *addrInputRegPort2 , Tm_outOfSetLoop,  Tm_outOfRtnLoop, Tm_pulseSent, echoHigh_1,  echoLow_1, Tm_start1, Tm_end1, echoHigh_2,  echoLow_2,  Tm_start2, Tm_end2, Tm_roundtrip1, Tm_roundtrip2, Dist1, Dist2");

}     // END of initialization


// Main loop runs continuously after setup
void loop() {
  // Small delay to allow ultrasonic waves to clear
  delay(20);

  // Clear values for this ping cycle
  Tm_start1 = 0;
  Tm_start2 = 0;
  Tm_end1 = 0;
  Tm_end2 = 0;
  Tm_roundtrip1 = 0;
  Tm_roundtrip2 = 0;
  dist1 = 0;
  dist2 = 0;
  echoHigh_1 = false;
  echoHigh_2 = false;
  echoLow_1 = false;
  echoLow_2 = false;

  // If either Echo input pin is still high (true/anything but 0), wait
  while ( ((bitMask1 & *addrInputRegPort1)) || ((bitMask2 & *addrInputRegPort2))) {
    delayMicroseconds(250);   // Wait 1/4000 second before checking again
  }

  // Basic sequence to signal both HC-SR04s to Transmit
  digitalWrite(TRIG_PIN1, LOW);   // Set trigger pin of both modules LOW
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(4);           // Hold for 4 millionths of a second
  digitalWrite(TRIG_PIN1, HIGH);  // Set trigger pin of both modules HIGH
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(16);          // Hold for 16 millionths of a second
  digitalWrite(TRIG_PIN1, LOW);   // Set trigger pin of both modules LOW
  digitalWrite(TRIG_PIN2, LOW);

  // Timestamp of when the pulse was requested - for timeout calculation
  Tm_pulseSent = micros();

  // Transition-to-high timestamp setting loop
  // Keep checking the echo pins while they are not both high and time is within reason
  while ( !(echoHigh_1 && echoHigh_2) && ( (micros() - Tm_pulseSent) < ( MAX_READY_WAIT ) )) {

    // Check if input register port BITWISE-ANDed with bitmask
    // is true (Echo pin 1 has gone high) AND the echo pin 1 high flag is not yet set
    if ( (!echoHigh_1) && (bitMask1 & *addrInputRegPort1)) {
      // Set module 1's beginning timestamp and set the started flag to true
      Tm_start1 = micros();
      echoHigh_1 = true;
    }
    // Check if (input register port BITWISE-ANDed with bitmask)
    // is true (Echo pin 2 has gone high) AND the echo pin 2 high flag is not yet set
    if ( (!echoHigh_2) && (bitMask2 & *addrInputRegPort2)) {
      // Set module 2's beginning timestamp and set the started flag to true
      Tm_start2 = micros();
      echoHigh_2 = true;
    }
  }   // End of Transition-to-high timestamp loop

  // Timestamp when the Echo going high loop was exited
  Tm_outOfSetLoop = micros();

  // Transition-to-low timestamp setting loop
  // If either of the Echo pins went high - meaning they sucessfully started listening
  if ( echoHigh_1 || echoHigh_2) {

    // Keep checking the echo pins while they are not both low and time is within reason
    while ( !(echoLow_1 && echoLow_2) && ((micros() - Tm_pulseSent) < ( MAX_ECHO_WAIT ) )) {

      // If the Echo pin 1 sucessfully went high AND
      // input register port BITWISE-ANDed with bitmask is false (pin has gone low)
      // AND the echo pin 1 low flag is not yet set
      if ( ( (echoHigh_1) && (!echoLow_1) && !(bitMask1 & *addrInputRegPort1)) ) {
        // Set the first module ending timestamp and set the ended flag to true
        Tm_end1 = micros();
        echoLow_1 = true;
      }

      // If the Echo pin 2 sucessfully went high AND
      // Input register port BITWISE-ANDed with bitmask is false (pin has gone low)
      // AND the echo pin 2 low flag is not yet set
      if ( (echoHigh_2) && (!echoLow_2) && !(bitMask2 & *addrInputRegPort2) ) {
        // Set the second module ending timestamp and set the ended flag to true
        Tm_end2 = micros();
        echoLow_2 = true;
      }

    }   // End of Transition-to-low timestamp loop

    // If module 1 started listening
    if (echoHigh_1) {
      // If module 1 heard an echo, calculate the roundtrip time
      if (echoLow_1) {
        Tm_roundtrip1 = Tm_end1 - Tm_start1;
      }
      // If module 1 did not hear an echo, set roundtrip time to maximum
      else {
        Tm_roundtrip1 = MAX_ECHO_WAIT;
      }
      // One way distance is approx equal to roundtrip time / 58
      dist1 = Tm_roundtrip1 / 58;
    }

    // Same sequence for module 2
    if (echoHigh_2) {
      if (echoLow_2) {
        Tm_roundtrip2 = Tm_end2 - Tm_start2;
        dist2 = Tm_roundtrip2 / 58;
      } else {
        Tm_roundtrip2 = MAX_ECHO_WAIT;
      }
      dist2 = Tm_roundtrip2 / 58;
    }
  }

  // Timestamp when the Echo going low loop was exited - for debugging
  Tm_outOfRtnLoop = micros();


  // WORK WITH DISTANCE VALUES TO CLEAN UP BAD DATA AND DISCREPANCIES
  // If there are distance values for both of the modules
  if (dist1 && dist2) {
    // If the two distances agree (are within 50cm of each other)
    if ( abs(dist1 - dist2) < 50 ) {
      // Add the distances to the distances array and increment index
      distances[index] = dist1;
      index++;
      distances[index] = dist2;
      index++;
    } // If there are two values, but they differ more than 50cm
    else {
      // Place the larger value into the distance array and increment the counter
      if (dist1 >= dist2) {
        distances[index] = dist1;
      }
      else {
        distances[index] = dist2;
      }
      index++;
    }
  } // If two values were not set then place either one or none into the array
  else {
    if (dist1) {
      distances[index] = dist1;
      index++;
    }
    else if (dist2) {
      distances[index] = dist2;
      index++;
    }
  }

  // If there are at least the required number of values in the array, average them
  if (index >= ELEMENTS) {
    for (int i = 0; i < ELEMENTS; i++) {
      sum += distances[i];
    }
    // Calculate average and then reset variables
    average = sum / ELEMENTS;
    index = 0;
    sum = 0;
    Tm_2 = millis();


    // If the distance indicates the path is clear of obstacles, set motor intensity low
    if ((average <= 0) || average >= 274) {
      intensity = 50;
    }
    else if (average < 80) {
      intensity = 255;
    }
    else if (average < 120) {
      intensity = 200;
    }
    else if (average < 160) {
      intensity = 150;
    }
    else if (average < 200) {
      intensity = 110;
    }
    else if (average < 240) {
      intensity = 75;
    }
    else if (average < 274) {
      intensity = 65;
    }

    analogWrite(MOTOR_PWM_PIN, intensity);

    //        // Output distance in CM to LCD for testing purposes
    //        lcd.setCursor(8,0);
    //        lcd.print("   ");
    //        lcd.setCursor(8,0);
    //        lcd.print(average);
    //
    //        lcd.setCursor(8,1);
    //        lcd.print("    ");
    //        lcd.setCursor(8,1);
    //        lcd.print(Tm_2-Tm_1);

    Tm_1 = Tm_2;    // Move end of this averaging cycle to beginning of next
  }

  //    //**** Uncomment block in order to output CSV data to serial ****
  //
  //    Serial.print( intensity);    Serial.print(" ,   ");
  //    Serial.print( port1);        Serial.print(" , ");
  //    Serial.print( bitMask1);     Serial.print(" , ");
  //
  //    Serial.print( port2);        Serial.print(" , ");
  //    Serial.print( bitMask2);     Serial.print(" , ");
  //
  //    Serial.print(* addrInputRegPort1);     Serial.print(" , ");
  //    Serial.print(* addrInputRegPort2);     Serial.print(" , ");
  //    Serial.print( Tm_outOfSetLoop);        Serial.print(" , ");
  //    Serial.print( Tm_outOfRtnLoop);        Serial.print(" , ");
  //    Serial.print( Tm_pulseSent);           Serial.print(" , ");
  //
  //    Serial.print(echoHigh_1);      Serial.print(" , ");
  //    Serial.print(echoLow_1);       Serial.print(" , ");
  //    Serial.print( Tm_start1);      Serial.print(" , ");
  //    Serial.print( Tm_end1);        Serial.print(" , ");
  //    Serial.print(echoHigh_2);      Serial.print(" , ");
  //    Serial.print(echoLow_2);       Serial.print(" , ");
  //    Serial.print( Tm_start2);      Serial.print(" , ");
  //    Serial.print( Tm_end2);        Serial.print(" , ");
  //
  //    Serial.print( Tm_roundtrip1);  Serial.print(" , ");
  //    Serial.print( Tm_roundtrip2);  Serial.print(" , ");
  //
  //    Serial.print( dist1);        Serial.print(" , ");
  //    Serial.print( dist2);        Serial.println("");


}  // END of main loop
