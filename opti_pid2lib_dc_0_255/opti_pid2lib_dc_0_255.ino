#include <Arduino.h>

//3598
#include "arduino2.h" 
#include <PID_v1.h>

//#define PID_END ( (PiDoutput==0 || PiDoutput==2.00) && (Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1))  )
#define PID_END (Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1))  //checking if position is reached
#define step_delay 100 //delay betwen steps in go_to_through function




#define leftstop 100            // Left  boundary
#define rightstop 3450           // Right  boundary
#define IN1 7
#define IN2 8
#define PWM 11
#define encoder1PinA 2      // X-AXIS  encoder 1 on pins 2 and 4
#define encoder1PinB 4
#define endstopX 3 //left end stop
//values 139.2
#define leftstop 100            // Left  boundary
#define rightstop 3450           // Right  boundary
#define deadzone 139.5 //dc motor dead zone - depend on pwm frequency and voltage135.5
#define err 60   //
#define respeed 180  //#define respeed 180

//////////Kp=1.42    err 0.8     deadzone 142.5 
//////////Kp=1.2    err 2.5     deadzone 143.5 
//////////Kp=2    err 0.5     deadzone 137.5
//////////Kp=2    err 10     deadzone 139.5
int newpos = 0;                    // Taget position for carriage
int* a;///////////dynamic array
volatile int encoder1Pos = 0;
//0.0000057886

////////////////////PID
//Define Variables we'll be connecting to
double Setpoint, Input, PiDoutput;

//Specify the links and initial tuning parameters
//double Kp=0.21, Ki=0.0000000001, Kd=0;
// Kp=0.0215, Ki=0.0000000001, Kd=0
//double Kp=1.42, Ki=0, Kd=0; Kp=1.25
double Kp=2, Ki=0, Kd=0;

//Kp=0.317, Kp=0.32 Ki=0, Kd=0;  /////def freq
PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, DIRECT);
//SetControllerDirection(DIRECT); 


void setup() {
//pwm pin d9 &d10
//TCCR1B = TCCR1B & B11111000 | B00000001;   // set timer 1 divisor to  1 for PWM frequency of 31372.55 Hz 

//pwm pin d3 &d11 timer2
TCCR2B = TCCR2B & 0b11111000 | B00000001;

  Serial.begin(9600);
  Serial.println("Linear Encoder Test  ");


pinMode2(encoder1PinA, INPUT);      // sets pin A as input
digitalWrite2(encoder1PinA, HIGH);  // turn on pullup resistors
pinMode2(encoder1PinB, INPUT);      // sets pin B as input
digitalWrite2(encoder1PinB, HIGH);  // turn on pullup resistors
pinMode2(endstopX, INPUT);  //endstop
digitalWrite2(endstopX, HIGH);

attachInterrupt(0, doEncoder1, CHANGE);  // encoder pin on interrupt 0 (pin 2)
attachInterrupt(1, doendstopX, FALLING); // endstop pin on interrupt 1 (pin 3)

//randomSeed(analogRead(0));

/////////PID
//initialize the variables we're linked to
//turn the PID on
myPID.SetMode(AUTOMATIC);
//myPID.SetSampleTime(2);
myPID.SetSampleTime(1);
myPID.SetOutputLimits(-(255-deadzone-err),(255-deadzone-err)); //+155 deadzone  +140 deadzone
//myPID.SetOutputLimits(-165, 165);//90 is added later//31khz 155
//myPID.SetOutputLimits(-115, 115);

Setpoint=newpos=500;

reset_pos(); //getting starting position

}

void loop() {
Input = encoder1Pos;
  
/*if (Pos1 != oldPos1){
     Serial.print("Encoder 1=");
     Serial.println(Pos1,DEC);
     Serial.print(PiDoutput);
     Serial.print("\t");
     oldPos1 = Pos1;
     }*/

if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    
    if (incomingByte=='q'){
      run_remembered();
    }
    else if (incomingByte=='c'){
      counting();
    }

    else if (incomingByte=='s'){
     
      show_count();
    }
    else if (incomingByte=='m'){
      
     newpos+=50;
     set_setpoint(newpos);
     go();
    }
    else if (incomingByte=='r'){
     
      move_dc_right(respeed);
      delay(1000); 
    }
    else if (incomingByte=='l'){
     
      move_dc_left(respeed);
      delay(1000); 
    }

    else if (incomingByte=='g'){
      go_to(encoder1Pos+36);
      delay(1000); 
    }


    
    else {
    //newpos =  random(300,3000);
    //newpos+=36;
    //go_to(newpos);
    go_to_through(2000,36);
    go_to_through(1000,36);
   // Serial.print("lel");
    //Serial.print("newpos ");
   // Serial.println(newpos,DEC);
   // Serial.print("setpoint ");
   // Serial.println(Setpoint,DEC);
    }
}   

 //Serial.println(Serial.available(),DEC);

//go();

}

/*void go(){
myPID.Compute();
PiDoutput > 0 ? move_dc_right(abs(PiDoutput)+deadzone) : move_dc_left(abs(PiDoutput)+deadzone);
}




void go(){
myPID.Compute();
if(encoder1Pos!=Setpoint){
if (PiDoutput>0){
  if (encoder1Pos<=120){
    move_dc_right(165);
      }
  else{
    move_dc_right(abs(PiDoutput)+deadzone); ///+90 deadzone- def freq// 140-31khz
        }
        }      
else {
//if (PiDoutput<-0.2){
    if (encoder1Pos>=3000){
      move_dc_left(165);//130
      }
    else{
    move_dc_left(abs(PiDoutput)+deadzone); 
      }
      }
//else {
//  dc_stop();
//}
}}




*/

void go(){
myPID.Compute();
//if(encoder1Pos!=Setpoint){
//if (PiDoutput>2.00){
if (PiDoutput>0.0000001){

    move_dc_right(abs(PiDoutput)+deadzone); ///+90 deadzone- def freq// 140-31khz
}      
else if (PiDoutput<-0.0000001){
    move_dc_left(abs(PiDoutput)+deadzone); 
}
//else if (Setpoint==(encoder1Pos)){
 //else { dc_stop();
 //return 1;
//}
}


void doEncoder1() {                     // ************** X- AXIS ****************
   if(digitalRead2(encoder1PinA) == HIGH){
       (digitalRead2(encoder1PinB) == LOW) ? (encoder1Pos = ++encoder1Pos) : (encoder1Pos = --encoder1Pos);
   }
   else {
      (digitalRead2(encoder1PinB) == HIGH) ? (encoder1Pos = ++encoder1Pos) : (encoder1Pos = --encoder1Pos);
   }
}
/*void doEncoder1() {         // ************** X- AXIS ****************
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
*/


//endstop interrupt functon
void doendstopX(){
  dc_stop();
  delay(500);
  encoder1Pos=0;
  Serial.print("stop \r\n");
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

 //    newpos =  random(100,maxi);
  //    Setpoint=newpos;
}

//getting starting position
void reset_pos(){
  Serial.print("resetting \r\n");
  move_dc_right(respeed);
  delay(350);

  move_dc_left(respeed);
  delay(1000);
  delay(1000);
  delay(1000);
 // while(encoder1Pos!=0){
 // move_dc_right(150);
//  }
  //delay(1000);
 //while(Setpoint!=encoder1Pos ||Setpoint!=(encoder1Pos-1)|| Setpoint!=(encoder1Pos-2)){
 //while(Setpoint!=(encoder1Pos ||(encoder1Pos-1))){
go_to(1500);
}


void set_setpoint(double set){
  if (set<leftstop){
   // dc_stop();
    Serial.print("left softstop\r\n");
  }

  else if (set>rightstop){
   // dc_stop();
    Serial.print("right softstop\r\n");
  }
  else Setpoint=set;
}


//counting number of steps on position belt
void counting(){

  reset_pos();
  move_dc_right(respeed+10);
  delay(100);
  Serial.print("counting \r\n");
  for (int i=0;i<9;i++){
  Serial.println(i,DEC);
     delay(100);
  }

  Serial.println(encoder1Pos,DEC);
  
}

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

void run_remembered(){
remember_pos();
for(int i=0;i<(sizeof(a)/sizeof(int));i++){
  Setpoint=a[i];
  go();
  delay(2000);
}
}

void show_count(){

 //  int incomingByte = Serial.read();
 // while(incomingByte=='s'){
Serial.print("encoder position: ");
  Serial.println(encoder1Pos,DEC);
  Serial.print("\r\n");
  //      Serial.print("pidout\n");
   //   Serial.println(PiDoutput,12);
    //   Serial.print("pidout+dead\n");
   //   Serial.println(PiDoutput+deadzone,12);
}



void go_to(int pointt){
Serial.print("going to pos ");
Serial.println(pointt,DEC);
Serial.print("\r\n");
 PiDoutput=0.000001;
set_setpoint(pointt);
//while(!(Setpoint==encoder1Pos || Setpoint==(encoder1Pos-1))){
int n=0;
//  pid loop 
while(!check_pid_end(PiDoutput)){//130
  Input = encoder1Pos;
  go();

 // if(n!=0 || !PID_END ){
 //   n++;
 // }
 // Serial.println(encoder1Pos,DEC);
//Serial.println(PiDoutput,12);
}
dc_stop();
//Serial.println("pid_end()");
//Serial.println(encoder1Pos,DEC);
//Serial.println(PiDoutput,12);
}



void go_to_through(int ppoint, int stepp){  //gout to ppoint by step which can skip exact position value
Serial.print("going to pos ");
Serial.println(ppoint,DEC);
Serial.print(" by ");
Serial.println(stepp,DEC);
Serial.print(" step");
Serial.print("\r\n");
 PiDoutput=0.000001;
if(ppoint>encoder1Pos){
  Serial.print("lel2");
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


bool check_pid_end(int x){
int x1=PiDoutput;
//int x2=PiDoutput;
if ((x==x1==PiDoutput) || PID_END) return true;
else return false;
}
