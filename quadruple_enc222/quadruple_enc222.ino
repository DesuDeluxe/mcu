#include <Arduino.h>

/* Based on Oleg Mazurov's code for rotary encoder interrupt service
   routines for AVR micros, originally written for ATmega644p:
   https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
   Here, implemented on Atmega328.

Example using Adafruit 377 rotary encoder: 
With the three encoder tabs facing downward and the knob towards you
* leftmost pin - connect to digital pin 3
* middle pin - connect to ground
* rightmost pin - connect to digital pin 2

*/

#define ENC_RD  PIND  //encoder port read
#define A_PIN 2  // pdip 4, associated with INT0 vector; PD2
#define B_PIN 3  // pdip 5, associated with INT1 vector; PD3

long counter = 0;

void setup() {
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  attachInterrupt(0, evaluateRotary, CHANGE);
  attachInterrupt(1, evaluateRotary, CHANGE);
  Serial.begin(9600); //make sure it's set to 9600 in the serial monitor on PC
  Serial.println("Start");
}

void loop() {
  static long lastCounter = 0;

  if(counter != lastCounter){
    Serial.print("Counter value: ");
    Serial.println(counter, DEC);
    lastCounter = counter;
    
  }

}

void evaluateRotary() {
/* encoder routine. Expects encoder with four state changes between detents */
/* and both pins open on detent */

  static uint8_t old_AB = 3;  //lookup table index
  static int8_t encval = 0;   //encoder value  
  static const int8_t enc_states [] PROGMEM = 
  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table
  /**/
  old_AB <<=2;  //remember previous state   old_AB |= (( ENC_RD >>2 ) & 0x03 );
  encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));

  if( encval > 3 ) {  //four steps forward
    counter++;
    encval = 0;
  }
  else if( encval < -3 ) {  //four steps backwards
   counter--;
   encval = 0;
  }
}

