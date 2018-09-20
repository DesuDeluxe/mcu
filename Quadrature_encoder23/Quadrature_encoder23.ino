#include <Arduino.h>

#define  GPIO2_PREFER_SPEED    1
#include "arduino2.h" 

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.
 
// Quadrature encoders
//  encoder
#define c_EncoderInterruptA 0
#define c_EncoderInterruptB 1
#define c_EncoderPinA 2
#define c_EncoderPinB 3
//#define EncoderIsReversed
volatile bool _EncoderASet;
volatile bool _EncoderBSet;
volatile bool _EncoderAPrev=0;
volatile bool _EncoderBPrev=0;
volatile long _EncoderTicks = 0;

//step pulse signal


#define IN1 6
#define IN2 7
#define PWM 14 
void setup()
{

    Serial.begin(9600);
 // Quadrature encoders
  //encoder
  pinMode2(c_EncoderPinA, INPUT);      // sets pin A as input
  digitalWrite2(c_EncoderPinA, LOW);  // turn on pullup resistors
  pinMode2(c_EncoderPinB, INPUT);      // sets pin B as input
  digitalWrite2(c_EncoderPinB, LOW);  // turn on pullup resistors
  attachInterrupt(c_EncoderInterruptA, HandleInterruptA, CHANGE);
//  attachInterrupt(c_EncoderInterruptB, HandleInterruptB, CHANGE);
//  attachInterrupt(PULSE_interrupt, move_dc, HIGH);

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

 
  delay(10);
  if (Serial.available()){
  Serial.print(_EncoderTicks);
     Serial.print("\t");
     delay(50);
  }
}

 
// Interrupt service routines for the  motor's quadrature encoder
void HandleInterruptA()
{
  _EncoderBSet = digitalRead2(c_EncoderPinB);
  _EncoderASet = digitalRead2(c_EncoderPinA);
  
  _EncoderTicks+=ParseEncoder();
  
  _EncoderAPrev = _EncoderASet;
  _EncoderBPrev = _EncoderBSet;




/*   
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _EncoderBSet = digitalRead2(c_EncoderPinB);   // read the input pin B

  // and adjust counter + if A leads B
  #ifdef EncoderIsReversed
    _EncoderTicks -= _EncoderBSet ? -1 : +1;  // if _EncoderBSet is set then we add or subtract  _EncoderTicks 
  #else
    _EncoderTicks += _EncoderBSet ? -1 : +1;
  #endif
/*  if (_EncoderBSet==1){
     _EncoderTicks -=1;
  }
  else if (_EncoderBSet==0){
     _EncoderTicks +=1;
  }
  */
  
  
}


int ParseEncoder(){
  if(_EncoderAPrev && _EncoderBPrev){
    if(!_EncoderASet && _EncoderBSet) return 1;
    if(_EncoderASet && !_EncoderBSet) return -1;
  }else if(!_EncoderAPrev && _EncoderBPrev){
    if(!_EncoderASet && !_EncoderBSet) return 1;
    if(_EncoderASet && _EncoderBSet) return -1;
  }else if(!_EncoderAPrev && !_EncoderBPrev){
    if(_EncoderASet && !_EncoderBSet) return 1;
    if(!_EncoderASet && _EncoderBSet) return -1;
  }else if(_EncoderAPrev && !_EncoderBPrev){
    if(_EncoderASet && _EncoderBSet) return 1;
    if(!_EncoderASet && !_EncoderBSet) return -1;
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
