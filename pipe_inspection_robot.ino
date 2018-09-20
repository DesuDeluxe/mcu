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

volatile int 1count;
volatile int 2mcount;
volatile int 3count;
volatile int 4count;


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


int SPEED=190; // PWM, fixed for now

int count;


//for xbee communication
int incomingByte = 0; 


//PID controller
double Setpoint, Input, PiDoutput;
double Kp=1, Ki=1, Kd=1;
PID myPID(&Input, &PiDoutput, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {                
//counting encoder signals
1count=0;
2count=0;
3count=0;
4count=0;


pinMode22(dir1,OUTPUT);
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

attachInterrupt(0, 1handleEncoder, CHANGE);// one side
attachInterrupt(1, 2handleEncoder, CHANGE);
attachInterrupt(2, 3phandleEncoder, CHANGE)//other
attachInterrupt(3, 4handleEncoder, CHANGE);

Serial2.begin(9600);//xbee
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
  if (Serial2.available() > 0) {
   
   incomingByte = Serial2.read();               
  }
  
  if(incomingByte == 10 ) forward(); 
  else if(incomingByte == 12) backward();
  else if(incomingByte == 14) pause();//stop

 
 incomingByte = 1;

 
 }
 
//one side of robot, 2 wheels
void 1handleEncoder()
{
if(digitalRead2(encoder1A) == digitalRead2(encoder1B))
{ 1count++;
}
else
{ 1count--;
}
}

void 2handleEncoder()
{
if(digitalRead2(encoder2A) == digitalRead2(encoder2B))
{ 2count++;
}
else
{ 2count--;
}
}



//other side
void 3handleEncoder()
{
if(digitalRead2(encoder3A) == digitalRead2(encoder3B))
{ 3count++;
}
else
{ 3count--;
}
}

void 4handleEncoder()
{
if(digitalRead2(encoder4A) == digitalRead2(encoder4B))
{ 4count++;
}
else
{ 4count--;
}
}


void forward()
{             

   
   analogWrite(speed1,0);
   analogWrite(speed2,0);
   analogWrite(speed3,0);
   analogWrite(speed4,0);
 
   delay(100);
 
   digitalWrite2(dir1,LOW);
   digitalWrite2(dir2,LOW); 
   digitalWrite2(dir3,HIGH);
   digitalWrite2(dir4,HIGH);
   analogWrite(speed1,SPEED);
   analogWrite(speed2,SPEED);
   analogWrite(speed3,SPEED);
   analogWrite(speed4,SPEED);
   

}

void backward()
{  

   analogWrite(speed1,0);
   analogWrite(speed2,0);
   analogWrite(speed3,0);
   analogWrite(speed4,0);
   
   delay(100);
 
   digitalWrite2(dir1,HIGH);
   digitalWrite2(dir2,HIGH); 
   digitalWrite2(dir3,LOW);
   digitalWrite2(dir4,LOW);
   analogWrite(speed1,SPEED);
   analogWrite(speed2,SPEED);
   analogWrite(speed3,SPEED);
   analogWrite(speed4,SPEED);


   

}

//stop
void pause()
{ 

  
   analogWrite(speed1,0);
   analogWrite(speed2,0);
   analogWrite(speed3,0);
   analogWrite(speed4,0);
  
 
}
