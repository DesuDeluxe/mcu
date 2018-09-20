#include <Arduino.h>

#include <Servo.h>

// Title:   Auto Pan-Tilt Servo/Cam Control
// Subject: This Sketch receives X,Y coordinates from srial then 
//          moves the camera to center of those coordinates. 
// Remixed: TechBitar / Hazim Bitar
// Date:    Dec 2011
// Credit:  I based this sketch on zagGrad's (SparkFun) code.  

#define  servomaxx   120   // max degree servo horizontal (x) can turn
#define  servomaxy   120   // max degree servo vertical (y) can turn
#define  screenmaxx   320   // max screen horizontal (x)resolution
#define  screenmaxy   240    // max screen vertical (y) resolution
#define  servocenterx   90  // center po#define  of x servo
#define  servocentery   90  // center po#define  of y servo
#define  servopinx   9   // digital pin for servo x
#define  servopiny   10  // digital servo for pin y

#define distancex 2  // x servo rotation steps
#define distancey 1  // y servo rotation steps

int valx = 0;       // store x data from serial port
int valy = 0;       // store y data from serial port
int posx = 0;
int posy = 0;
int incx = 20;  // significant increments of horizontal (x) camera movement
int incy = 20;  // significant increments of vertical (y) camera movement
int x=0,y=0;

Servo servox;
Servo servoy;

char pitch = 0;
char yaw = 1;

char serialChar=0;

void setup() {

  Serial.begin(9600);        // connect to the serial port
  Serial.println("Starting Cam-servo Face tracker");

  pinMode(servopinx,OUTPUT);   
  pinMode(servopiny,OUTPUT);   

  servoy.attach(servopiny); 
  servox.attach(servopinx); 

  // center servos

  servox.write(servocenterx); 
  delay(200);
  servoy.write(servocentery); 
  delay(200);
}

void loop () {
  
posx = servox.read(); 
posy = servoy.read();

if (Serial.available() >0){  //Wait for a character on the serial port.
serialChar=Serial.read();
///////////////X - pitch
if(serialChar  == pitch){  //Check to see if the character is the servo ID for the tilt servo
  while(Serial.available() <=0);//Wait for the second command byte from the serial port.
  
  valx=Serial.read();
  //Find out if the X component of the face is to the left of the middle of the screen.
  if(valx  < (screenmaxx/2 - incx)){
    if( posx  <=(servomaxx)){
      posx += distancex;
      x=1;    
    } //Update the pan position variable to move the servo to the left.
  }
    //Find out if the X component of the face is to the right of the middle of the screen.
  else if(valx > (screenmaxx/2 + incx)){
    if(posx >=20){
      posx -=distancex; 
      x=2;
    }//Update the pan position variable to move the servo to the right.
  }

servox.write(posx);
delay(15);
//delay(200);
}
///////////////Y - yaw
else if(serialChar == yaw){ //Check to see if the initial serial character was the servo ID for the pan servo.
  while(Serial.available() <= 0);  //Wait for the second command byte from the serial port.

  valy=Serial.read();
  //Find out if the Y component of the face is below the middle of the screen.
  if(valy < (screenmaxy/2 - incy)){
    if(posy <=(servomaxy)){
      posy += distancey;
      y=1;
    } //If it is below the middle of the screen, update the tilt position variable to lower the tilt servo.
  }
    //Find out if the Y component of the face is above the middle of the screen.
  else if(valy > (screenmaxy/2 + incy)){
    if(posy >=20){posy -= distancey;
    y=2;
    } //Update the tilt position variable to raise the tilt servo.
  }

 // if (posy<=20)
 // {
 // servoy.write(20);
//  }
//  else{
servoy.write(posy);
delay(15);
//delay(200);
 //   }
//for (int i=servoy.read();i<=posy;i++){
  //  servoy.write(i);
  //  Serial.print(i);
   // delay(200);
  //   Serial.print ("ardy"); 
  //  Serial.print(posy);
}


}

/*else{
  switch (x){
  case 1:
  posx += distancex;
servox.write(posx);
  delay(200);
break;
  case 2:
  posx -= distancex;
servox.write(posx);
  delay(200);
break;
 }
  switch (y){
  case 1:
if(posy <=(servomaxy-incy)){posy += distancey;}
 servoy.write(posy);
   delay(200);
break;
  case 2:
if(posy >=20){posy -= distancey;}
 servoy.write(posy);
   delay(200);
break;
  }
 

}
*/
}

