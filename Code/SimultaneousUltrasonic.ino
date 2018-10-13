// Simultaneous Ultrasonic Readings
// by G. C. Holden
// Allows for improved consistancy in ultrasonic reading by utilizing  
// two inexpensive HC-SR04 modules pointed in the same direction and
// manually pulsing them simultaneously and using timestamps (no timers)
// to calculate distances. 

// Must include the Arduino Library 
#include <Arduino.h>
#include <LiquidCrystal.h> 

#define TRIG_PIN1          26    // Pin # connected to Ultrasonic Module 1 Trigger
#define ECHO_PIN1          27    // Pin # connected to Ultrasonic Module 1  Echo

#define TRIG_PIN2          30    // Pin # connected to Ultrasonic Module 2 Trigger
#define ECHO_PIN2          31    // Pin # connected to Ultrasonic Module 2 Echo

#define MOTOR_PWM_PIN      2    // PWM Pin # connected to Motor control

#define ELEMENTS           4    // Number of elements to average together for distance reading

#define MAX_ECHO_WAIT   15892   // Maximum time to wait for echo pins to return low (distance of 274 cm * 58 microseconds seconds per cm) 
#define MAX_READY_WAIT  550     // Maximum number of microseconds to wait for echo pins to be high (ready)

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Setup LCD display object

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
    
    uint32_t Tm_1;      // Timestamp of beginning of one averaging cycle
    uint32_t Tm_2;      // Timestamp of end of one averaging cycle
    
    uint32_t Tm_roundtrip1;    // Microseconds until ping returns
    uint32_t Tm_roundtrip2;    // Microseconds until ping returns

    volatile uint8_t* addrInputRegPort1;  // Address of the register holding the input state for port 1 
    volatile uint8_t* addrInputRegPort2;  // Address of the register holding the input state for port 2
                                          // Declared volatile in case ultrasonic module changes the state while temporarily stored
    
    int dist1 = 0;                    // Temporary storage of calculated distance 1
    int dist2 = 0;                    // Temporary storage of calculated distance 2
    int distances[ELEMENTS+2] = {0};  // Array holding validated distance calculations 
    int index = 0;                    // Array indexing
    int sum = 0;                      // Accumulator for averaging distances
    int average = 0;                  // Average of validated distances
    int intensity = 0;                // Motor intensity


void setup() {          // Initialization routine only runs once 
    // Define pin modes
    pinMode(TRIG_PIN1, OUTPUT);
    pinMode(ECHO_PIN1, INPUT);
    pinMode(TRIG_PIN2, OUTPUT);
    pinMode(ECHO_PIN2, INPUT);

    pinMode(MOTOR_PWM_PIN, OUTPUT);

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

    // USE LCD DISPLAY FOR TESTING OF VALUES
    // Initialize LCD display
    lcd.begin(16, 2);              // Define 16 character x 2 line
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Booting up...."); // print a simple message
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Dist 1:     cm");
    lcd.setCursor(0,1);
    lcd.print("Elapse:     millis");
    Tm_1 = millis();
    Tm_2 = millis();
    
//    //**** Uncomment these in order to output CSV Header to serial monitor **** 
//    // Start Serial communication
//    Serial.begin(115200);
//    // Wait 1 second to enable activation of Serial Monitor
//    delay(1000);//    Serial.println( "port1 , bitMask1,  port2,  bitMask2, *addrInputRegPort1, *addrInputRegPort2 , 
//    Tm_outOfSetLoop,  Tm_outOfRtnLoop, Tm_pulseSent, echoHigh_1,  echoLow_1, Tm_start1, Tm_end1, 
//    echoHigh_2,  echoLow_2,  Tm_start2, Tm_end2, Tm_roundtrip1, Tm_roundtrip2");
  
} // END of initialization 



void loop() {  
      // Small delay to allow ultrasonic waves to clear
      delay(5);
      
      // If either pin is still high (true), wait
      while ( ((bitMask1 & *addrInputRegPort1)) || ((bitMask2 & *addrInputRegPort2))){
        delayMicroseconds(500);
      }

      // Clear values for this ping cycle
      Tm_roundtrip1 = 0;
      Tm_roundtrip2 = 0;
      dist1 = 0;
      dist2 = 0;
      echoHigh_1 = false;
      echoHigh_2 = false;
      echoLow_1 = false;
      echoLow_2 = false;

      // Basic sequence to signal both HC-SR04s to Transmit
      digitalWrite(TRIG_PIN1, LOW);
      digitalWrite(TRIG_PIN2, LOW);
      delayMicroseconds(4);
      digitalWrite(TRIG_PIN1, HIGH);
      digitalWrite(TRIG_PIN2, HIGH);
      delayMicroseconds(10);  
      digitalWrite(TRIG_PIN1, LOW);
      digitalWrite(TRIG_PIN2, LOW);
      // Timestamp of when the pulse was requested
      Tm_pulseSent = micros();

      // Keep checking the echo pins while they are not both high and time is within reason
      while ( !(echoHigh_1 && echoHigh_2) && ( (micros() - Tm_pulseSent) < ( MAX_READY_WAIT ) )) {
        // Check if input register port BITWISE-ANDed with bitmask is true (pin has gone high)
        // AND the echo pin high 1 flag is not yet set
        if ( ((bitMask1 & *addrInputRegPort1)) && (!echoHigh_1)) {
          // Set the first module beginning timestamp and set the started flag to true
          Tm_start1 = micros();
          echoHigh_1 = true;
        }
        // Check if input register port BITWISE-ANDed with bitmask is true (pin has gone high)
        // AND the echo pin high 2 flag is not yet set
        if ( ((bitMask2 & *addrInputRegPort2)) && (!echoHigh_2)) {
          // Set the second module beginning timestamp and set the started flag to true
          Tm_start2 = micros();
          echoHigh_2 = true;
        }
      }
      // Timestamp when the Echo going high loop was exited
      Tm_outOfSetLoop = micros();

      // If either of the Echo pins went high - meaning they sucessfully started listening 
      if ( echoHigh_1 || echoHigh_2) {

        // Keep checking the echo pins while they are not both low and time is within reason        
        while ( !(echoLow_1 && echoLow_2) && ((micros() - Tm_pulseSent) < ( MAX_ECHO_WAIT ) )) {
          
          // Check if input register port BITWISE-ANDed with bitmask is false (pin has gone low)
          // AND the echo pin low 1 flag is not yet set
          if ( ( !(bitMask1 & *addrInputRegPort1)) && (!echoLow_1)) {            
            // Set the first module ending timestamp and set the ended flag to true  
            Tm_end1 = micros();
            echoLow_1 = true;
          }
          
          // Check if input register port BITWISE-ANDed with bitmask is false (pin has gone low)
          // AND the echo pin low 1 flag is not yet set          
          if ( ( !(bitMask2 & *addrInputRegPort2)) && (!echoLow_2)) {
          // Set the second module ending timestamp and set the ended flag to true            
            Tm_end2 = micros();
            echoLow_2 = true;
          }

        } 
        // If module 1 started listening
        if (echoHigh_1) {
          // If module 1 heard an echo, calculate the roundtrip time
          if (echoLow_1) { Tm_roundtrip1 = Tm_end1 - Tm_start1; } 
          // If module 1 did not hear an echo, set roundtrip time to maximum
          else { Tm_roundtrip1 = MAX_ECHO_WAIT;  }
        // One way distance is approx equal to roundtrip time / 58
        dist1 = Tm_roundtrip1/58;
        }

        // Same sequence for module 2  
        if (echoHigh_2) {
          if (echoLow_2) { 
            Tm_roundtrip2 = Tm_end2 - Tm_start2;
            dist2 = Tm_roundtrip2/58;
          } else { 
            Tm_roundtrip2 = MAX_ECHO_WAIT;
          }
        dist2 = Tm_roundtrip2/58;
        }
      }

      // Timestamp when the Echo going low loop was exited
      Tm_outOfRtnLoop = micros();


      // WORK WITH DISTANCE VALUES TO CLEAN UP BAD DATA AND DISCREPANCIES
      // If there are distance values for both of the modules
      if (dist1 && dist2) {
        // If the two distances are within 50cm of eachother
        if ( abs(dist1 - dist2) < 50 ) {
          // Add the distances to the array of distances and increment index
          distances[index] = dist1;
          index++;
          distances[index] = dist2;
          index++;
        } // If there are two values, but they differ more than 50cm
        else {
          // Place the larger value into the distance array and increment the counter
          if (dist1 >= dist2) { distances[index] = dist1; }
          else { distances[index] = dist2; }
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
        for (int i = 0; i < ELEMENTS; i++){ sum += distances[i]; }
        
        // Calculate average and then reset variables
        average = sum / ELEMENTS;
        index = 0;
        sum = 0;
        Tm_2 = millis();

        // Output distance in CM to LCD for testing purposes
        lcd.setCursor(8,0);  
        lcd.print("   ");
        lcd.setCursor(8,0);
        lcd.print(average);  
  
        lcd.setCursor(8,1);  
        lcd.print("    ");
        lcd.setCursor(8,1);
        lcd.print(Tm_2-Tm_1);
        Tm_1 = Tm_2;
      }

            

//    //**** Uncomment these in order to output CSV data to serial ****
//
//    Serial.print( port1);        Serial.print(" , ");
//    Serial.print( bitMask1);        Serial.print(" , ");
//    
//    Serial.print( port2);        Serial.print(" , ");
//    Serial.print( bitMask2);        Serial.print(" , ");
//
//    Serial.print(* addrInputRegPort1);        Serial.print(" , "); 
//    Serial.print(* addrInputRegPort2);        Serial.print(" , "); 
//    Serial.print( Tm_outOfSetLoop);        Serial.print(" , "); 
//    Serial.print( Tm_outOfRtnLoop);        Serial.print(" , "); 
//    Serial.print( Tm_pulseSent);     Serial.print(" , "); 
//
//    Serial.print(echoHigh_1);        Serial.print(" , ");    
//    Serial.print(echoLow_1);        Serial.print(" , ");    
//    Serial.print( Tm_start1);        Serial.print(" , ");  
//    Serial.print( Tm_end1);        Serial.print(" , ");   
//    Serial.print(echoHigh_2);        Serial.print(" , ");   
//    Serial.print(echoLow_2);        Serial.print(" , ");    
//    Serial.print( Tm_start2);        Serial.print(" , ");    
//    Serial.print( Tm_end2);        Serial.print(" , ");  
//    
//    Serial.print( Tm_roundtrip1/58);        Serial.print(" , ");   
//    Serial.print( Tm_roundtrip2/58);        Serial.print(" , "); 
//                  

}  // END of main loop
