#include <Arduino.h>

/*a4988
//Quarter step

48 steps for full rev

gear ratio 36:16 = 2.25

full step   2.25 angle 7.5
half step - 4.5 angle 3.75
quarter     9 angle 1.875     
eight       18 angle 0.9375  <=== default
sixteenth   36 angle 0.46875


*/

const int ledPin = 13; // the pin that the LED is attached to
int incomingByte;      // a variable to read incoming serial data into
const int stepPin = 3; 


void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(stepPin,OUTPUT); 
}

void loop() {
if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'S') {
      digitalWrite(ledPin, HIGH);
      pulse(1);

    }
 if (incomingByte == 'K') {
      digitalWrite(ledPin, LOW);
      pulse(2);
    }
}
}

void pulse(int stepnum){
 for(int x = 0; x < stepnum; x++) {
    digitalWrite(stepPin,HIGH);
     delay(30);
   // delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delay(30);
  
}
}

