//#include <Arduino.h>

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

//pins
#define IN1 7
#define IN2 8
#define PWM 11 //timer2
#define encoder1PinA 2      // X-AXIS  encoder 1 on pins 2 and 4
#define encoder1PinB 4
#define endstopXL 3 //left end stop

//helpers
#define check_ if(Serial.available() >0) return;
#define PID_END (Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1))  //checking if position is reached
#define leftstop 200            // Left  boundary
#define rightstop 3300           // Right  boundary
#define centerpos rightstop/leftstop/2
#define deadzone 139.5    //dc motor dead zone - depend on pwm frequency and voltage 140.5, 139.5
#define err 70  //limit pid max output 60
#define respeed 175  //reset position speed
#define distancelin 10
#define searchinc 36 //increemnt of go_to_through function
#define step_delay 100 //delay betwen steps in go_to_through function
//search servo signall delay
#define  sdlay  50 //stages functions
#define  sdlayall 200 //all()
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC

//#define  foor(a,b,c)  for(int a=b;i>=;i--){
#define  servomaxx   180   // max degree servo horizontal (x) can turn
#define  servomaxy   118   // max degree servo vertical (y) can turn
#define  servominy   40   // min degree servo vertical (y) can turn
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

////////////////////PID
double Setpoint, Input, PiDoutput;
double Kp=2, Ki=0, Kd=0;
//double Kp=0.2, Ki=0, Kd=0; deadzone 0

PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, DIRECT);
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC

void setup() {
  TCCR2B = TCCR2B & 0b11111000 | B00000001; //timer2 pwm frequency -31372.55Hz  pin d3 &d11

  Serial.begin(9600);        // connect to the serial port
  Serial.println("Linear Servo Test ");

  pinMode2(servopinx,OUTPUT);
  pinMode2(servopiny,OUTPUT);

  servoy.attach(servopiny);
  servox.attach(servopinx);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
pinMode2(encoder1PinA, INPUT);      // sets  encoder pin A as input
digitalWrite2(encoder1PinA, HIGH);  // turn on pullup resistor
pinMode2(encoder1PinB, INPUT);      // sets  encoder pin B as input
digitalWrite2(encoder1PinB, HIGH);  // turn on pullup resistor
pinMode2(endstopXL, INPUT);  /// sets endstop pin as input
digitalWrite2(endstopXL, HIGH); // turn on pullup resistor

attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
attachInterrupt(1, doendstopXL, FALLING); // endstop pin on interrupt 1 (pin 3)

randomSeed(analogRead(0));
//turn the PID on
myPID.SetSampleTime(1);
myPID.SetOutputLimits(-(255-deadzone-err),(255-deadzone-err)); //limiting pid output to defined err value and deadzone
reset_pos(); //getting starting position
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
// center servos
servox.write(servocenterx);
delay(200);
servoy.write(servocentery);
delay(200);
Serial.print("face fallower start \r\n");
}



void loop () {
Input = encoder1Pos; //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
go();
posx = servox.read();
posy = servoy.read();
//go();
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
      Serial.print("dc left \r\n");
      servoy.write(90);
      setpointvar-=distancelin;
      set_setpoint(setpointvar);
     }
    x=1;
    }

  servox.write(posx);
  go();
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
  else if (serialChar=='s'){
    show_count();
  }

}
else {
  Serial.print("face lost\r\n");
  find_face();
}

}//main loop end



        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC
void go(){
myPID.Compute();
//if (PiDoutput>2.00){
if (PiDoutput>0.0000001){ move_dc_right(abs(PiDoutput)+deadzone); ///+90 deadzone- def freq// 140-31khz
}
else if (PiDoutput<-0.0000001){ move_dc_left(abs(PiDoutput)+deadzone);
}
//else { dc_stop();}
}

void doEncoder1() {                     // ************** X- AXIS ****************
   if(digitalRead2(encoder1PinA) == HIGH){
       (digitalRead2(encoder1PinB) == LOW) ? (encoder1Pos = ++encoder1Pos) : (encoder1Pos = --encoder1Pos);
   }
   else {
      (digitalRead2(encoder1PinB) == HIGH) ? (encoder1Pos = ++encoder1Pos) : (encoder1Pos = --encoder1Pos);
   }
}

//endstop interrupt functon
void doendstopXL(){
  dc_stop();
  encoder1Pos=0;
  Serial.print("endstop stop \r\n");
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
  delay(350);
  move_dc_left(respeed);
  delay(1000);
  delay(1000);
  delay(1000);
  go_to(centerpos);
}


void set_setpoint(double set){ //helper function to not exceed limits
  if (set<leftstop){
    Serial.print("left softstop\r\n");
  }
  else if (set>rightstop){
    Serial.print("right softstop\r\n");
  }
  else Setpoint=set;
}


void find_face(){
Serial.print("searching\r\n");

all();
((x) == 0 ? (stage1right()) : (stage1left()));
stage2();
if (Serial.available() >0) return;
//left
go_to(leftstop);
stage1();
stage2();
if (Serial.available() >0) return;
//right
go_to(rightstop);
stage1();
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
Serial.print("stage 1r -horizontal \r\n");
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
Serial.print("stage 1l -horizontal \r\n");
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
Serial.print("stage 2 -horizontal + vertical \r\n");
//stage 2 - right
//for (int i=90,j=90.g=servomaxy;i<=servomaxx,j<=servomaxy,g>=servominy;i++,j+=2){
for (int i=90;i<=servomaxx;i++){
 if(i<=servomaxy)servoy.write(i/2);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
}
//stage 2 - center
for (int i=servomaxx;i>=0;i--){

 if(i<=servomaxy && i>servominy)servoy.write(i/2);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
 }
}


void all(){
  int inc=leftstop;
 while (Serial.available() ==0){
Serial.print("infinite loop to find face \r\n");
//stage 2 - right
Serial.print("right \r\n");
for (int i=90;i<=servomaxx;i++){
  inc+=searchinc;
go_to(inc);
 if(i<=servomaxy)servoy.write(i);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlayall);
}
Serial.print("left \r\n");
//stage 2 - center
inc=rightstop;
for (int i=servomaxx;i>=0;i--){
  inc-=searchinc;
go_to(inc);
 if(i<=servomaxy && i>servominy)servoy.write(i);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlayall);
}
}
return;
}

void show_count(){
Serial.print("encoder position: "); Serial.println(encoder1Pos,DEC); Serial.print("\r\n");
Serial.print("pidout "); Serial.println(PiDoutput,12);
}

void go_to(int pointt){
Serial.print("going to pos ");Serial.println(pointt,DEC);Serial.print("\r\n");
//?????????????PiDoutput=0.000001;  //needed to get into while loop
set_setpoint(pointt);
//  pid loop
while(!check_pid_end(PiDoutput)){ Input = encoder1Pos; go();
}
dc_stop();
}

void go_to_through(int ppoint, int stepp){  //gout to ppoint by step which can skip exact position value
Serial.print("going to pos ");Serial.println(ppoint,DEC);Serial.print(" by ");Serial.println(stepp,DEC);Serial.print(" step\r\n");
//?????????????PiDoutput=0.000001;  //needed to get into while loop
if(ppoint>encoder1Pos){
  while( (ppoint>=encoder1Pos) ){
  go_to(encoder1Pos+stepp);
  delay(step_delay);
  }
}
else if(ppoint<encoder1Pos){
  while( (ppoint<=encoder1Pos) ){
  go_to(encoder1Pos-stepp);
  delay(step_delay);
  }
}
}

//to find out if position is obtained
bool check_pid_end(int x){
int x1=PiDoutput;
//int x2=PiDoutput;
if ((x==x1==PiDoutput) || PID_END) return true;
else return false;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DC


/*void stage2(){
  if (Serial.available() >0) return;
Serial.print("stage 2 -horizontal + vertival \r\n");
//stage 2 - right
//for (int i=90,j=90;i<=servomaxx,j<=servomaxy;i++,j+=3){
for (int i=90;i<=servomaxx;i++){
 if(i<=servomaxy)servoy.write(i);
 servox.write(i);
 if (Serial.available() >0) return;
  delay(sdlay);
}*/
