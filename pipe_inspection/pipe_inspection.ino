#include <Arduino.h>

  /***************************************************************
  * Service Robots
  * 
  * Pipe inspection robot
  * 
  * 
  * driving with 4 dc motors and 4 encoders
  * 
  * motor control board has 4 quadrature mixing circuits 
  * - takes the 2 inputs from a quadrature encoder
  * and mixes them into a single output
  * 
  * 
  * communicating with use of xbee module
  * 
  * 
  * 
  * 
  * 
  * !
  * speed calculation or tracking position is not done, and is not feeded to PID to controll motors
  * 
  * 
  *****************************************************************/
#include "arduino2.h" //arduino fast pin reading library
#include <PID_v1.h>  //pid library



//motor board interrupt pins
#define encoder1 2
#define encoder2 3
#define encoder3 18
#define encoder4 19

//encoder pins to obtain speed and direction 
#define encoder1A 22
#define encoder1B 23 
#define encoder2A 24
#define encoder2B 25
#define encoder3A 26
#define encoder3B 27
#define encoder4A 28
#define encoder4B 29

//counting encoder signals
volatile int count1=0;
volatile int count2=0;
volatile int count3=0;
volatile int count4=0;


//pwm pins
int speed1 = 9; 
int speed2 = 10; 
int speed3 = 11; 
int speed4 = 12; 

// direction
int dir1 = 44; 
int dir2 = 45; 
int dir3 = 42; 
int dir4 = 43; 

//distance
int dist=0;
int count1prev=0;
int count2prev=0;
int count3prev=0;
int count4prev=0;

int SPEED=190; // PWM, fixed for now

int add_left=0;
int add_right=0;

//for xbee communication
int incomingByte = 0; 

//PID controller
double Setpoint, Input, PiDoutput;
double Kp=1, Ki=1, Kd=1;
PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {                



pinMode2(dir1,OUTPUT);
pinMode2(dir2,OUTPUT);
pinMode2(dir3,OUTPUT);
pinMode2(dir4,OUTPUT);

pinMode2(speed1,OUTPUT);
pinMode2(speed2,OUTPUT);
pinMode2(speed3,OUTPUT);
pinMode2(speed4,OUTPUT);

pinMode2(encoder1A, INPUT);
pinMode2(encoder1B, INPUT);
pinMode2(encoder2A, INPUT);
pinMode2(encoder2B, INPUT);
pinMode2(encoder3A, INPUT);
pinMode2(encoder3B, INPUT);
pinMode2(encoder4A, INPUT);
pinMode2(encoder4B, INPUT);

attachInterrupt(0, handleEncoder1, CHANGE);// one side
attachInterrupt(1, handleEncoder2, CHANGE);
attachInterrupt(2, handleEncoder3,CHANGE);//other
attachInterrupt(3, handleEncoder4, CHANGE);

Serial.begin(9600);//xbee
//Serial.begin(9600);

analogWrite(speed1,0);
analogWrite(speed2,0);
analogWrite(speed3,0);
analogWrite(speed4,0);



//not used for now
//setting up PID
myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(2);
myPID.SetOutputLimits(-255, 255) ;


}


void loop() {

  //xbee
  if (Serial.available() > 0) {
   
   incomingByte = Serial.read();               
  }
  
  if(incomingByte == 10 ) forward(); 
  else if(incomingByte == 12) backward();
  else if(incomingByte == 14) pause();//stop

 
 incomingByte = 1;

 
 }
 
//one side of robot, 2 wheels
void handleEncoder1()
{
if(digitalRead2(encoder1A) == digitalRead2(encoder1B))
{ count1++;
}
else
{ count1--;
}
}

void handleEncoder2()
{
if(digitalRead2(encoder2A) == digitalRead2(encoder2B))
{ count2++;
}
else
{ count2--;
}
}



//other side
void handleEncoder3()
{
if(digitalRead2(encoder3A) == digitalRead2(encoder3B))
{ count3++;
}
else
{ count3--;
}
}

void handleEncoder4()
{
if(digitalRead2(encoder4A) == digitalRead2(encoder4B))
{ count4++;
}
else
{ count4--;
}
}


void forward()
{             
if_roll();
   
   analogWrite(speed1,0);
   analogWrite(speed2,0);
   analogWrite(speed3,0);
   analogWrite(speed4,0);
 
   delay(100);
 
   digitalWrite2(dir1,LOW);
   digitalWrite2(dir2,LOW); 
   digitalWrite2(dir3,HIGH);
   digitalWrite2(dir4,HIGH);
   analogWrite(speed1,SPEED+add_left);
   analogWrite(speed2,SPEED+add_left);
   analogWrite(speed3,SPEED+add_right);
   analogWrite(speed4,SPEED+add_right);
   

}

void backward()
{  
  if_roll();

   analogWrite(speed1,0);
   analogWrite(speed2,0);
   analogWrite(speed3,0);
   analogWrite(speed4,0);
   
   delay(100);
 
   digitalWrite2(dir1,HIGH);
   digitalWrite2(dir2,HIGH); 
   digitalWrite2(dir3,LOW);
   digitalWrite2(dir4,LOW);
   analogWrite(speed1,SPEED+add_left);
   analogWrite(speed2,SPEED+add_left);
   analogWrite(speed3,SPEED+add_right);
   analogWrite(speed4,SPEED+add_right);


   

}

//stop
void pause()
{ 

  
   analogWrite(speed1,0);
   analogWrite(speed2,0);
   analogWrite(speed3,0);
   analogWrite(speed4,0);
  
 
}

//measurring distance robot travelled
int distance(){
  
  if (count1>count1prev && count2>count2prev && count3>count3prev && count4>count4prev){
  dist++;
  }
  
  else if (count1<count1prev && count2<count2prev && count3<count3prev && count4<count4prev){
  dist--;
  }
  
  count1prev=count1;
  count2prev=count2;
  count3prev=count3;
  count4prev=count4;
                  
  int distance_in_cm=(dist/(full revolution counts)*(wheel circumference);

  return distance_in_cm;
}

//if robot rolls in the pipe, add speed to one side of wheels
void if_roll(){
  
  int roll=get_roll();
  if (roll>15){
    add_left=30;
    }
  else if (roll<-15){
    add_right=30;
    }
  else{
    add_right=0;
    add_left=0;
    }
}

//calculating roll angle from the IMU sensor
int get_roll(){

}

