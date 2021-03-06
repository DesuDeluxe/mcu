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

#define frontstop = 100            // Right most encoder boundary
#define backstop = 1700            // Left most encoder boundary
#define IN1 8
#define IN2 7
#define PWM 9 

const int encoder1PinA = 2;        // X-AXIS  encoder 1 on pins 2 and 4
const int encoder1PinB = 4;
volatile int encoder1Pos = 0;


boolean CarriageDir = 0;           // Carriage Direction '0' is Right to left
byte spd = 250;                    // Carriage speed from 0-255
int newpos = 100;                    // Taget position for carriage
int posrchd = 1;                   // Flag for target reached

int Pos1,maxi=3455;


void setup() {
  Serial.begin(9600);
  Serial.println("Linear Encoder Test  04-29-2014");

pinMode2(encoder1PinA, INPUT);      // sets pin A as input
digitalWrite2(encoder1PinA, HIGH);  // turn on pullup resistors
pinMode2(encoder1PinB, INPUT);      // sets pin B as input
digitalWrite2(encoder1PinB, HIGH);  // turn on pullup resistors
  
//  move_dc_left(spd)        // Bring carriage to home position. 

  delay(100); 
//  move_dc_right(0)        // Bring carriage to home position. 

  
  attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
  // attachInterrupt(1, doEncoder2, CHANGE);  // encoder pin on interrupt 1 (pin 3)
  
  randomSeed(analogRead(0));
}

void loop() {
static int oldPos1, oldPos2;
uint8_t oldSREG = SREG;

uint8_t i;

  cli();
  Pos1 = encoder1Pos;  
 
  SREG = oldSREG;
  
  if(Pos1 != oldPos1){
     Serial.print("Encoder 1=");
     Serial.println(Pos1,DEC);
              Serial.print(newpos);
    Serial.print("\t");
     oldPos1 = Pos1;
  }
  

  

  //sweep_carriage();
  
  if(posrchd) {                           // If target has been reached clear flag, and get new target
 //   newpos =  random(100,maxi);
    posrchd = 0;
  }    
    
  posrchd = go_to_target(newpos);
  
}


/***************************************************************************************
The following code was taken from   http://forum.arduino.cc/index.php?topic=41615.20;wap2
to utilize the fast port based encoder logic.  Thank you Lefty!
please go there for a full explanation of how this works.  I have truncated the comments 
here for brevity.
***************************************************************************************/
void doEncoder1() {                                  // ************** X- AXIS ****************
   if(digitalRead2(encoder1PinA) == HIGH)
   {
    if(digitalRead2(encoder1PinB) == LOW)
    {
       encoder1Pos = ++encoder1Pos;
    }
    else
    {
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
// End of interrupt code for encoder #1
                                                   




/***************************************************************************************
go_to_target() determines the distance and direction from current position to target 
position, then maps speed to decellerate close to the target so as not to overshoot.
***************************************************************************************/


int go_to_target(int target)
{
  int temp = 0;
  int delta = abs(Pos1-target);                   // Distance to target
  spd = map(delta,maxi,0,250,100);                // Decellerate as you get closer
  if(target < maxi && target > 50) {
     if(Pos1 < target) {
       move_dc_right(spd);
       temp = 0;
     } else if(Pos1 > target) {
       move_dc_left(spd);

       temp = 0;
     }  else{
      dc_stop();
      temp =1;
     }
  }
  return temp;
}




//void move_dc_left(int pid)
void move_dc_right(int pid)
{
   digitalWrite2(IN1, HIGH);
    digitalWrite2(IN2, LOW);
     analogWrite(PWM, pid);
 
  }

//void move_dc_right(int pid)
void move_dc_left(int pid)
{
   digitalWrite2(IN1, LOW);
    digitalWrite2(IN2, HIGH);
     analogWrite(PWM, pid);
 
  }
void dc_stop()
{
   digitalWrite2(IN1, LOW);
    digitalWrite2(IN2, LOW);
  //   analogWrite(PWM, 255);
  analogWrite(PWM, 0);
 
  }
