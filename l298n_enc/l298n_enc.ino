#include <Arduino.h>

//Code by Reichenstein7 (thejamerson.com)

//Keyboard Controls:
//
// 1 -Motor 1 Left
// 2 -Motor 1 Stop
// 3 -Motor 1 Right


// Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.

#define  GPIO2_PREFER_SPEED    1
#include "arduino2.h" 

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.
 
// Quadrature encoders
// Left encoder
#define c_EncoderInterruptA 0
#define c_EncoderInterruptB 1
#define c_EncoderPinA 2
#define c_EncoderPinB 3
volatile bool _EncoderAPrev=0;
volatile bool _EncoderBPrev=0;
//#define EncoderIsReversed
volatile bool _EncoderBSet;
volatile long _EncoderTicks = 0;






int dir1PinA = 8;
int dir2PinA = 7;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed

void setup() {  // Setup runs once per reset
// initialize serial communication @ 9600 baud:
Serial.begin(9600);


  pinMode2(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite2(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode2(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite2(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_EncoderInterruptA, HandleInterruptA, RISING);


//Define L298N Dual H-Bridge Motor Controller Pins

pinMode(dir1PinA,OUTPUT);
pinMode(dir2PinA,OUTPUT);
pinMode(speedPinA,OUTPUT);


}

void loop() {

// Initialize the Serial interface:

if (Serial.available() > 0) {
int inByte = Serial.read();
int speed; // Local variable

switch (inByte) {

//______________Motor 1______________

case '1': // Motor 1 Forward
analogWrite(speedPinA, 190);//Sets speed variable via PWM 
digitalWrite(dir1PinA, LOW);
digitalWrite(dir2PinA, HIGH);
Serial.println("Motor 1 Forward"); // Prints out “Motor 1 Forward” on the serial monitor
Serial.println("   "); // Creates a blank line printed on the serial monitor
break;

case '2': // Motor 1 Stop (Freespin)
analogWrite(speedPinA, 0);
digitalWrite(dir1PinA, LOW);
digitalWrite(dir2PinA, HIGH);
Serial.println("Motor 1 Stop");
Serial.println("   ");
break;

case '3': // Motor 1 Reverse
analogWrite(speedPinA, 190);
digitalWrite(dir1PinA, HIGH);
digitalWrite(dir2PinA, LOW);
Serial.println("Motor 1 Reverse");
Serial.println("   ");
break;

case '4': // tick
Serial.print("\t");
Serial.print(_EncoderTicks);
break;


default:
// turn all the connections off if an unmapped key is pressed:
for (int thisPin = 2; thisPin < 11; thisPin++) {
digitalWrite(thisPin, LOW);
}
  }
    }
      }





      
void HandleInterruptA()
{

  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _EncoderBSet = digitalRead2(c_EncoderPinB);   // read the input pin B

  // and adjust counter + if A leads B
  #ifdef EncoderIsReversed
    _EncoderTicks -= _EncoderBSet ? -1 : +1;  // if _EncoderBSet is set then we add or subtract  _EncoderTicks 
  #else
    _EncoderTicks += _EncoderBSet ? -1 : +1;
  #endif
  }
/*  if (_EncoderBSet==1){
     _EncoderTicks -=1;
  }
  else if (_EncoderBSet==0){
     _EncoderTicks +=1;
  }
  */
  
  
}

