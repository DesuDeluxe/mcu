#include <Arduino.h>

 /***************************************************************************************
*  Lin_Enc_01.ino   04-29-2014   unix_guru at hotmail.com   @unix_guru on twitter
*  http://arduino-pi.blogspot.com
*
*  This sketch allows you to run two salvaged printer carriages for X/Y axis using their 
*  linear encoder strips for tracking. 
*  Both interrupt routines are below, but for the sake of demonstration, I've only set up
*  the X-AXIS for now.
*
*****************************************************************************************/
#include "arduino2.h" 
#include <PID_v1.h>

#define frontstop = 100            // Right most encoder boundary
#define backstop = 1700            // Left most encoder boundary
#define IN1 8
#define IN2 7
#define PWM 9
#define DIR 6
#define STEP 5

const int encoder1PinA = 2;        // X-AXIS  encoder 1 on pins 2 and 4
const int encoder1PinB = 4;
volatile int encoder1Pos = 0;
const int endstopX = 3; //end stop

//////////
boolean CarriageDir = 0;           // Carriage Direction '0' is Right to left
byte spd = 250;                    // Carriage speed from 0-255
int newpos = 0;                    // Taget position for carriage
int posrchd = 1;                   // Flag for target reached
////////////////

int Pos1,maxi=2200;

////////////////////PID
//Define Variables we'll be connecting to
double Setpoint, Input, PiDoutput;

//Specify the links and initial tuning parameters
double Kp=0.315, Ki=0, Kd=0;

PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, REVERSE);
//SetControllerDirection(DIRECT); 


void setup() {
  Serial.begin(9600);
  Serial.println("Linear Encoder Test  ");


pinMode2(encoder1PinA, INPUT);      // sets pin A as input
digitalWrite2(encoder1PinA, HIGH);  // turn on pullup resistors

pinMode2(encoder1PinB, INPUT);      // sets pin B as input
digitalWrite2(encoder1PinB, HIGH);  // turn on pullup resistors

pinMode2(endstopX, INPUT);  //endstop
digitalWrite2(endstopX, HIGH);

pinMode2(DIR, INPUT); //direction input
digitalWrite2(DIR, LOW);

pinMode2(STEP, INPUT); //step input
digitalWrite2(STEP, LOW);

attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
attachInterrupt(1, doendstopX, FALLING); // endstop pin on interrupt 1 (pin 3)

//randomSeed(analogRead(0));

/////////PID
//initialize the variables we're linked to
//turn the PID on
myPID.SetMode(AUTOMATIC);
//myPID.SetSampleTime(2);
myPID.SetOutputLimits(-165, 165);//90 is added later

Setpoint=newpos=200;

reset_pos(); //getting starting position
  
}

void loop() {
static int oldPos1;
uint8_t oldSREG = SREG;

uint8_t i;

  cli();
 Pos1 = encoder1Pos;  
  SREG = oldSREG;
  
if (Pos1 != oldPos1){
     Serial.print("Encoder 1=");
     Serial.println(Pos1,DEC);
     Serial.print(PiDoutput);
     Serial.print("\t");
     oldPos1 = Pos1;
     }

if (Serial.available() > 0) {
 // if(posrchd) {                           // If target has been reached clear flag, and get new target
    int incomingByte = Serial.read();
    counting();
   // newpos =  random(100,maxi);
   // Setpoint=newpos;
    Serial.print("newpos ");
    Serial.println(newpos,DEC);
    Serial.print("setpoint ");
    Serial.println(Setpoint,DEC);
    posrchd = 0;
    }   
////////////////////////////////////////////////////STEP DIR
if(digitalRead2(STEP)==HIGH){
  if(digitalRead2(DIR)==HIGH){
  Setpoint=Input+300;  
  }
  else{
    Setpoint=Input-300;
  }
}
//////////////PID
Input = encoder1Pos;
myPID.Compute();
if (PiDoutput>0.5){
  if (encoder1Pos==50){
    move_dc_right(130);
      }
  else{
    move_dc_right(abs(PiDoutput)+90); ///+90 deadzone
        }}      
else if (PiDoutput<-0.5){
   // if (encoder1Pos==50){
   //   move_dc_left(130);
   //   }
  //  else{
    move_dc_left(abs(PiDoutput)+90); 
      }
    //  }
else {
  dc_stop();
  }
}

/***************************************************************************************
The following code was taken from   http://forum.arduino.cc/index.php?topic=41615.20;wap2
to utilize the fast port based encoder logic.  Thank you Lefty!
please go there for a full explanation of how this works.  I have truncated the comments 
here for brevity.
***************************************************************************************/
void doEncoder1() {         // ************** X- AXIS ****************
   if(digitalRead2(encoder1PinA) == HIGH){
      if(digitalRead2(encoder1PinB) == LOW){
        encoder1Pos = ++encoder1Pos;
        }
      else{
        encoder1Pos = --encoder1Pos;
        }
   }  
   else {
      if(digitalRead2(encoder1PinB)==HIGH){
        encoder1Pos = ++encoder1Pos;
        }
       else {
        encoder1Pos = --encoder1Pos;   
       }
   }

}

//endstop interrupt functon
void doendstopX(){
  dc_stop();
  encoder1Pos=0;
  Serial.print("stop");
}

void move_dc_left(float pid){
  digitalWrite2(IN1, HIGH);
   digitalWrite2(IN2, LOW);
   analogWrite(PWM, pid);
 
}

void move_dc_right(float pid){

   digitalWrite2(IN1, LOW);
   digitalWrite2(IN2, HIGH);
   analogWrite(PWM, pid);
 
  }
void dc_stop(){
   digitalWrite2(IN1, LOW);
   digitalWrite2(IN2, LOW);
    digitalWrite2(PWM, HIGH);

 //    newpos =  random(100,maxi);
  //    Setpoint=newpos;

}

//getting starting position
void reset_pos(){
  move_dc_left(130);
  delay(1000); 
  move_dc_right(130);
  delay(1000); 
}

//counting number of steps on position belt
void counting(){
  reset_pos()
  move_dc_right(100);
  delay(100000); 
}
