#include <Arduino.h>

//3598
#include "arduino2.h"
#include <PID_v1.h>

#define IN1 7
#define IN2 8
#define PWM 11
#define encoder1PinA 2      // X-AXIS  encoder 1 on pins 2 and 4
#define encoder1PinB 4
#define endstopX 3 //left end stop

//helpers
#define PID_END ( (PiDoutput==0 || PiDoutput==2.00) && (Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1)) )
//#define PID_END (Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1))  //checking if position is reached
//#define PID_END2(x) ( (PiDoutput==0 || PiDoutput==2.00) && x==PiDoutput  )
#define step_delay 100 //delay betwen steps in go_to_through function

//#define centerpos rightstop/leftstop/2
#define centerpos 1500
#define leftstop 100            // Left  boundary
#define rightstop 3450           // Right  boundary
#define deadzone 117 //dc motor dead zone - depend on pwm frequency and voltage 120
#define err 0   //limit pid max output60
#define respeed 150  //#define respeed 180


int newpos = 0;
//dynamic array TODO: int* a;
volatile long encoder1Pos = 0;

////////////////////PID
double Setpoint, Input, PiDoutput;
double Kp=0.28, Ki=0, Kd=0.0000000000000000000001;
//double Kp=0.5, Ki=0, Kd=0; 0.2
PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
TCCR2B = TCCR2B & 0b11111000 | B00000001; //timer2 pwm frequency -31372.55Hz  pin d3 &d11

Serial.begin(9600);
Serial.println("rotary Servo Test ");


pinMode2(encoder1PinA, INPUT);      // sets  encoder pin A as input
digitalWrite2(encoder1PinA, HIGH);  // turn on pullup resistor
pinMode2(encoder1PinB, INPUT);      // sets  encoder pin B as input
digitalWrite2(encoder1PinB, HIGH);  // turn on pullup resistor
pinMode2(endstopX, INPUT);  /// sets endstop pin as input
digitalWrite2(endstopX, HIGH); // turn on pullup resistor

attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
attachInterrupt(1, doendstopX, FALLING); // endstop pin on interrupt 1 (pin 3)

randomSeed(analogRead(0));
//turn the PID on
myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(1);
myPID.SetOutputLimits(-(255-deadzone-err),(255-deadzone-err)); //limiting pid output to defined err value and deadzone

//reset_pos(); //getting starting position
Setpoint=(encoder1Pos+500);
}
void loop() {
Input = encoder1Pos;
if (Serial.available() > 0) { //getting commands from pc
    char cmd = Serial.read();
    switch(cmd) {
      //case 'q': run_remembered(); break;
      case 'c': counting(); break;
     // case 'd': {Serial.print("deadzone speed \r\n");Serial.println(n,DEC);move_dc_right(n);delay(2000);n+=1;  // int n=120; first by 10 then by 1 then by 0.5/0.1 break;}
      case 's': show_count(); break;
      case 'm': Setpoint=(encoder1Pos+500); break;//newpos+=50; set_setpoint(newpos); go(); break;
      case 'r': move_dc_right(respeed); delay(1000);  break;
      case 'l': move_dc_left(respeed);  delay(1000); break;
      case 'g': go_to(encoder1Pos+36);  delay(1000);  break;
      case 'n': go_to(random(leftstop,rightstop));  break;
      default:  go_to_through(2000,36); go_to_through(1000,36); break;
    }

}
go();
//show_count();
}

void go(){
myPID.Compute();
PiDoutput > 0 ? move_dc_right(abs(PiDoutput)+deadzone) : move_dc_left(abs(PiDoutput)+deadzone);
}

/*
void go(){
myPID.Compute();
//if (PiDoutput>2.00){
if (PiDoutput>0.0000001){ move_dc_right(abs(PiDoutput)+deadzone); ///+90 deadzone- def freq// 140-31khz
}
else if (PiDoutput<-0.0000001){ move_dc_left(abs(PiDoutput)+deadzone);
}
//else { dc_stop();}
}*/

void doEncoder1() {                 // ************** X- AXIS ****************
   if(digitalRead2(encoder1PinA) == HIGH){
       (digitalRead2(encoder1PinB) == LOW) ? (encoder1Pos = ++encoder1Pos) : (encoder1Pos = --encoder1Pos);
   }
   else {
      (digitalRead2(encoder1PinB) == HIGH) ? (encoder1Pos = ++encoder1Pos) : (encoder1Pos = --encoder1Pos);
   }
}


//endstop interrupt functon
void doendstopX(){
  dc_stop();
  delay(1000); //debouncing
  encoder1Pos=0; //resetting encoder value
  Serial.print("stop \r\n");
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
  Serial.print("resetting... \r\n");
  move_dc_right(respeed);
  delay(350);
  move_dc_left(respeed);
  delay(1000);
  delay(1000);
  delay(1000);
 // go_to(centerpos);
 Setpoint=(encoder1Pos+500);
 show_count();
 counting();
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


//counting number of steps on position belt
void counting(){
 // reset_pos();
  move_dc_right(respeed+10);
  delay(100);
  Serial.print("counting \r\n");
  for (int i=0;i<9;i++){
  Serial.println(i,DEC);
  delay(100);
  }
  Serial.println(encoder1Pos,DEC);
}

/*TODO:
void remember_pos(){
if (Serial.available() > 0) {
Serial.print("how many positions \r\n");
int incomingByte = Serial.read();
Serial.println(incomingByte,DEC);
a = (int *)malloc(incomingByte * sizeof(int));
if (NULL == a) {
  Serial.print("malloc failed\n");
  return;
}
}
//a[0]=encoder1Pos;
for(int i=0;i<(sizeof(a)/sizeof(int));i++){
  Serial.print("move to pos and enter \r\n");
  while (Serial.read() != '\n') {
  }
  a[i]=encoder1Pos;
  }
}

//TODO:
void run_remembered(){
remember_pos();
for(int i=0;i<(sizeof(a)/sizeof(int));i++){
  Setpoint=a[i];
  go();
  delay(2000);
}
}
*/

void show_count(){
Serial.print("encoder position: "); Serial.println(encoder1Pos,DEC); Serial.print("\r\n");
Serial.print("pidout "); Serial.println(PiDoutput,12);
}


void go_to(int pointt){
Serial.print("going to pos ");Serial.println(pointt,DEC);Serial.print("\r\n");
//PiDoutput=0.000001;  //needed to get into while loop
set_setpoint(pointt);
//  pid loop
while(!PID_END){
Input = encoder1Pos; go();
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
bool check_pid_end(){
int x2=(Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1));
if ((x2==(Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1))) ) return true;
else return false;
}


