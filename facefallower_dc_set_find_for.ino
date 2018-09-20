#include <Arduino.h>

/*
 arduino obtains x and y coordinates by serial of center of rectangle produced by opencv on pc
then it calculates if these coordinates are off the center of screen
if off then it moves servos one desired angle at the time
if x servo max range is reached then it starts to move linear drive

servo library disables pwm on pins 9 and 10

 */
#include <Servo.h>
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
#include "arduino2.h"
#include <PID_v1.h>
//help values
#define searchinc 40
//pins
#define IN1 7
#define IN2 8
#define PWM 11 //timer2
#define encoder1PinA 2      // X-AXIS  encoder 1 on pins 2 and 4
#define encoder1PinB 4
#define endstopXL 3 //left end stop
//values
#define leftstop 100            // Left  boundary
#define rightstop 3450           // Right  boundary
#define deadzone 0     //dc motor dead zone - depend on pwm frequency and voltage
#define respeed 170  //#define respeed 190
#define distancelin 10
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC

//#define  foor(a,b,c)  for(int a=b;i>=;i--){
#define  sdlay  300
#define  sdlayall 200
#define  servomaxx   180   // max degree servo horizontal (x) can turn
#define  servomaxy   120   // max degree servo vertical (y) can turn
#define  screenmaxx   320   // max screen horizontal (x)resolution
#define  screenmaxy   240    // max screen vertical (y) resolution
#define  servocenterx   90  // center po#define  of x servo
#define  servocentery   90  // center po#define  of y servo
#define  servopinx   10   // digital pin for servo x
#define  servopiny   9  // digital servo for pin y
#define distancex 2  // x servo rotation steps
#define distancey 1  // y servo rotation steps
#define incx 20  // significant increments of horizontal (x) camera movement
#define incy 20  // significant increments of vertical (y) camera movement
int valx = 0;       // store x data from serial port
int valy = 0;       // store y data from serial port
int posx = 0;       // store x data from servo
int posy = 0;       // store y data from servo
double setpointvar = 1500;

Servo servox;
Servo servoy;

char pitch = 0;
char yaw = 1;
char serialChar=0;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC

bool x,y;//indicator values for direction 0 - right | 1 - left
volatile int encoder1Pos = 0;

//Define PID Variables we'll be connecting to
double Setpoint, Input, PiDoutput;
double Kp=0.2, Ki=0, Kd=0;

PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, DIRECT);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC

void setup() {
  TCCR2B = TCCR2B & 0b11111000 | B00000001;   //++++++DC   set timer  divisor to  1 for PWM frequency of 31372.55 hz

  Serial.begin(9600);        // connect to the serial port
  Serial.println("Starting Face tracker");

  pinMode2(servopinx,OUTPUT);
  pinMode2(servopiny,OUTPUT);

  servoy.attach(servopiny);
  servox.attach(servopinx);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
pinMode2(encoder1PinA, INPUT);      // sets pin A as input
digitalWrite2(encoder1PinA, HIGH);  // turn on pullup resistors
pinMode2(encoder1PinB, INPUT);      // sets pin B as input
digitalWrite2(encoder1PinB, HIGH);  // turn on pullup resistors
pinMode2(endstopXL, INPUT);  //endstop
digitalWrite2(endstopXL, HIGH);

pinMode2(IN1, OUTPUT);
pinMode2(IN2, OUTPUT);
pinMode2(PWM, OUTPUT);

attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
attachInterrupt(1, doendstopXL, FALLING); // endstop pin on interrupt 1 (pin 3)
/////////PID
myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(1);
//myPID.SetOutputLimits(-255,255);
myPID.SetOutputLimits(-210,210);
reset_pos(); //getting starting position
Setpoint=setpointvar=1500;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
// center servos
servox.write(servocenterx);
delay(200);
servoy.write(servocentery);
delay(200);
}



void loop () {
Input = encoder1Pos; //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC

posx = servox.read();
posy = servoy.read();
go();
if (Serial.available() >0){
  serialChar=Serial.read();
                    ///////////////X - pitch
  if(serialChar  == pitch){
    while(Serial.available() <=0);//Wait for the second command byte from the serial port.

    valx=Serial.read();
                    //Find out if the X component of the face is to the left of the middle of the screen.
    if(valx  < (screenmaxx/2 - incx)){
      if( (posx + distancex)  <=(servomaxx)){
        posx += distancex;
      }
                     //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
      else if (posx>servomaxx){
        Serial.print("dc right \r\n");
        setpointvar+=distancelin;
        set_setpoint(setpointvar);
      }             //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
    x=0;
    }
                   //Find out if the X component of the face is to the right of the middle of the screen.
    else if(valx > (screenmaxx/2 + incx)){
      if((posx -distancex) >=20){
        posx -=distancex;
      }
     else if (posx<20){
      Serial.print("dx left \r\n");
      servoy.write(90);
      setpointvar-=distancelin;
      set_setpoint(setpointvar);
     }
    x=1;
    }

  servox.write(posx);
//  go();
  delay(15);
  }
                  ///////////////Y - yaw
  else if(serialChar == yaw){ //Check to see if the initial serial character was the servo ID for the pan servo.
    while(Serial.available() <= 0);  //Wait for the second command byte from the serial port.
    valy=Serial.read();
    if(valy < (screenmaxy/2 - incy)){   //Find out if the Y component of the face is below the middle of the screen.
      if(posy <=(servomaxy)){posy += distancey; y=0;  }
    }
    else if(valy > (screenmaxy/2 + incy)){ //Find out if the Y component of the face is above the middle of the screen.
      if(posy >=20){posy -= distancey; y=1; }
    }
  servoy.write(posy);
  }

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
  else if (serialChar=='m'){
 // setpointvar+=50;
 // set_setpoint(setpointvar);
  //go();
  Serial.print("encpos ");
  Serial.println(encoder1Pos,DEC);
  Serial.print("setpoint ");
  Serial.println(Setpoint,DEC);
  Serial.print("PiDoutput ");
  Serial.println(PiDoutput,DEC);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
  }

}
else find_face();
/*else{//if face lost - move in same direction as last
    if (x==1){
    setpointvar+=distancelin;
    set_setpoint(setpointvar);
  //  go();
    }

    else if (x==2){
    setpointvar-=distancelin;
    set_setpoint(setpointvar);
 //   go();
    }

}*/

}//main loop end



        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
void go(){
myPID.Compute();
//if(encoder1Pos!=Setpoint){
((PiDoutput) > 0 ? (move_dc_right(abs(PiDoutput)+deadzone)) : (move_dc_left(abs(PiDoutput)+deadzone));
if (PiDoutput>0){
move_dc_right(abs(PiDoutput)+deadzone); ///+90 deadzone- def freq// 140-31khz
}
else {
move_dc_left(abs(PiDoutput)+deadzone);
}
}

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
void doendstopXL(){
  dc_stop();
  encoder1Pos=0;
  Serial.print("endstop stop \r\n");
  //Serial.println(encoder1Pos,DEC);
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
}

//getting starting position
void reset_pos(){
  Serial.print("resetting pos \r\n");
  move_dc_right(respeed);
  delay(500);

  move_dc_left(respeed);
  delay(1000);
  delay(1000);
  delay(1000);
}


void set_setpoint(double set){
  if (set<=leftstop){
    dc_stop();
    Serial.print("left softstop stop \r\n");
  }

  else if (set>=rightstop){
    dc_stop();
    Serial.print("right softstop stop \r\n");
  }
  else Setpoint=set;
}


void find_face(){
((x) == 0 ? (stage1right()) : (stage1left()));
if (Serial.available() >0) return;
stage2();
if (Serial.available() >0) return;
//left
set_setpoint(leftstop);
while(Setpoint!=encoder1Pos){
  Input = encoder1Pos;
  if (Serial.available() >0) return;
  go();
}
stage1();
if (Serial.available() >0) return;
stage2();
if (Serial.available() >0) return;
//right
set_setpoint(rightstop);
while(Setpoint!=encoder1Pos){
  Input = encoder1Pos;
  if (Serial.available() >0) return;
  go();
}
stage1();
if (Serial.available() >0) return;
stage2();
if (Serial.available() >0) return;

all();
return;
}



void stage1(){
 if (Serial.available() >0) return;
Serial.print("stage 1 -horizontal \r\n");
//stage 1 - center->right
for (int i=90;i<=servomaxx;i++){
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
}
//stage 1 - right->left
for (int i=servomaxx;i>=0;i--){
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
 }
}


void stage1right(){
 if (Serial.available() >0) return;
Serial.print("stage 1 -horizontal \r\n");
//stage 1 - right
for (int i=servox.read();i<=servomaxx;i++){
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
}
//stage 1 - center
for (int i=servomaxx;i>=0;i--){
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
 }
}


void stage1left(){
 if (Serial.available() >0) return;
Serial.print("stage 1 -horizontal \r\n");
//stage 1 - right
for (int i=servox.read();i>=0;i--){
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
}
//stage 1 - center
for (int i=0;i<=servomaxx;i++){
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
 }
}



void stage2(){
  if (Serial.available() >0) return;
Serial.print("stage 2 -horizontal + vertival \r\n");
//stage 2 - right
//for (int i=90,j=90;i<=servomaxx,j<=servomaxy;i++,j+=3){
for (int i=90;i<=servomaxx;i++){
 if(i<=servomaxy)servoy.write(i+3);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
}
//stage 2 - center
for (int i=servomaxx;i>=0;i--){

 if(i<=servomaxy)servoy.write(i+3);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
 }
}


void all(){
 while (Serial.available() ==0){
Serial.print("infinite loop to find face \r\n");
//stage 2 - right

for (int i=90;i<=servomaxx;i++){
Input = encoder1Pos;
set_setpoint(i*searchinc);
  go();
 if(i<=servomaxy)servoy.write(i+3);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlayall);
}
//stage 2 - center
for (int i=servomaxx;i>=0;i--){
Input = encoder1Pos;
  set_setpoint(i*searchinc);
  go();
 if(i<=servomaxy)servoy.write(i+3);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlayall);
}
}
return;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
