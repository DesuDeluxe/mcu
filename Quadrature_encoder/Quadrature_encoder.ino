#include <Arduino.h>

#define  GPIO2_PREFER_SPEED    1
#include "arduino2.h" 

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.
 
// Quadrature encoders
// Left encoder
#define c_EncoderInterrupt 0
#define c_EncoderPinA 2
#define c_EncoderPinB 2
//#define EncoderIsReversed
volatile boolean _EncoderBSet;
volatile long _EncoderTicks = 0;
byte newData = 0;
byte prevData = 0;

//step pulse signal
#define PULSE_interrupt 1

#define IN1 6
#define IN2 7
#define PWM 14 
void setup()
{
/////////////////////ADC
  cli();//disable interrupts
  
  //set up continuous sampling of analog pin 0
  
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;
  
  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements
  
  sei();//enable interrupts


//////////////////////////////

    Serial.begin(9600);
 // Quadrature encoders
  //encoder
 // pinMode2(c_EncoderPinA, INPUT);      // sets pin A as input
 // digitalWrite2(c_EncoderPinA, LOW);  // turn on pullup resistors
//  pinMode2(c_EncoderPinB, INPUT);      // sets pin B as input
  //digitalWrite2(c_EncoderPinB, HIGH);  // turn on pullup resistors
//  attachInterrupt(c_EncoderInterrupt, HandleInterruptA, CHANGE);
//////////////////////////  attachInterrupt(PULSE_interrupt, move_dc, HIGH);

/* //dc driver control
 pinMode2(PWM, OUTPUT);
 pinMode2(IN1, OUTPUT);
 pinMode2(IN2, OUTPUT);
 //dc initial stop
 analogWrite2(PWM, 0);
 digitalWrite2(IN1, LOW);
 digitalWrite2(IN2, LOW);
 */
}
 
void loop()
{

 
  delay(20);
 if(Serial.available()){
 Serial.print(_EncoderTicks);
 Serial.print("\t");
}
  
  
}

 
// Interrupt service routines for the left motor's quadrature encoder
/*void HandleInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
//  _EncoderBSet = analogRead(c_EncoderPinB);   // read the input pin B

  // and adjust counter + if A leads B
 /* #ifdef EncoderIsReversed
    _EncoderTicks -= _EncoderBSet ? -1 : +1;  // if _EncoderBSet is set then we add or subtract  _EncoderTicks 
  #else
    _EncoderTicks += _EncoderBSet ? -1 : +1;
  #endif
  if (_EncoderBSet==1){
     _EncoderTicks -=1;
  }
  else{
     _EncoderTicks +=1;
  }
  
}
*/

ISR(ADC_vect) {//when new ADC value ready

  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  newData += ADCH;
  newData = newData/2;
  if (newData > prevData || newData == 1023){//if increasing
 //   _EncoderBSet=1;//set pin 12 high
 _EncoderTicks +=1;
  }
  else if (newData == 0 ||newData < prevData){
   // _EncoderBSet=0;//set pin 12 low
   _EncoderTicks -=1;
  }

 
  
}










///////////////////dc controll
/*
void move_dc_left()
{
   digitalWrite2(IN1, HIGH);
    digitalWrite2(IN2, LOW);
     analogWrite2(PWM, PID);
 
  }

void move_dc_right()
{
   digitalWrite2(IN1, LOW);
    digitalWrite2(IN2, HIGH);
     analogWrite2(PWM, PID);
 
  }
void dc_stop()
{
   digitalWrite2(IN1, LOW);
    digitalWrite2(IN2, LOW);
     analogWrite2(PWM, 0);
 
  }*/
