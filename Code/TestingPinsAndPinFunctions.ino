// Pin Mapper
// by G. C. Holden

// Simple program to print the register and bitmask for pins
// based on hardware selected for compiling. 
// Outputs CSV of 8 bit binary values to the Arduino IDE
// Serial monitor (115200) where the list can be cut and pasted into
// a .CSV text file. It can then be opened in Excel, and 
// sorted and searched as needed.
// Runs a loop that increments from 1 to NUM_DIGITAL_PINS, which is
// defined within the Arduino Library.

// Must include the Arduino Library
#include <Arduino.h>

void setup() {          // Initialization routine only runs once 

    uint8_t pinToPort;
    uint8_t pinToBitMask;
    uint8_t portContent;

    uint8_t checkbit;
    int place;
    
    // Start Serial communication
    Serial.begin(115200);
    // Wait 3 seconds to enable activation of Serial Monitor
    delay(3000);


    // Print header
    Serial.println ("Pin, Port, Port-Pin, Regist #, Bitmask, Port Content");

    // Begin loop to index through pin numbers

    for (int index=0; index < NUM_DIGITAL_PINS; index++){


      // Print pin number
      Serial.print (index);
      Serial.print (", ");

      // Store value of the current numbered pins assigned register in pinToPort
      pinToPort = digitalPinToPort(index);
      // Store value of the current numbered pins register bitmask in pinToBitMask
      pinToBitMask = digitalPinToBitMask(index);
      
      // Test to see if pin number is an invalid pin
      if ((pinToPort > 0) & (pinToPort < (13))){

        // Check each bit place in bitmask to determine 
        // what the placement in the bitmask is
        
        checkbit = pinToBitMask;
        for (place = 7; place >= 0; place--){
          checkbit = pinToBitMask >> place;
          if (checkbit == 1) break;
        }
        
        // Print port and port-pin combination
        Serial.write (pinToPort + 64); // Shift into ASCII range to print registers as letters
        Serial.print (", ");
        Serial.write (pinToPort + 64);
        Serial.print (place);
        Serial.print (", ");
        Serial.print (pinToPort);
        Serial.print (", ");

        // Print bitmask for that pin location within its register 
        Serial.print (pinToBitMask,BIN);
        Serial.print (", ");

        // Print mode flags of register (status of each bit flag)
        portContent = *portModeRegister(pinToPort);
        Serial.print  (portContent,BIN);
        Serial.print (", ");

        // Print output mode flags of register (status of each bit flag)
        portContent = *portOutputRegister(pinToPort);
        Serial.print  (portContent,BIN);
        Serial.print (", ");

        // Print high/low values of input pins in register (status of each bit flag)
        portContent = *portInputRegister(pinToPort);
        Serial.println (portContent,BIN);
        
      } else {
        // Note that pin location was not a pin and move to next number
        Serial.println ("NOT A PIN");
      }

    }
    Serial.println ("\n\n\nEXECUTION COMPLETE.");
    Serial.println ("Use Ctrl-A, Ctrl-C to copy contents of serial monitor");  
    Serial.println ("Paste into a text file with .csv extension and save.");
    Serial.println ("Open with Excel and remove these lines. Sort as desired.");
    
} // END of initialization 



void loop() {          // Main loop does nothing
  delay(3000);
}  // END of main loop
