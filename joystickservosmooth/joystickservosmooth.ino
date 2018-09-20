#include <Arduino.h>

//source http://www.instructables.com/id/Arduino-2-Servos-Thumbstick-joystick/?ALLSTEPS

#include <Servo.h>

const int servo1 = 11;       // first servo
const int servo2 = 10;       // second servo
const int joyH = 3;        // L/R Parallax Thumbstick
const int joyV = 2;        // U/D Parallax Thumbstick

int servoVal1; 
int servoVal2;// variable to read the value from the analog pin

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

void setup() {

  // Servo  
  myservo1.attach(servo1);  // attaches the servo
  myservo2.attach(servo2);  // attaches the servo

  // Inizialize Serial
  Serial.begin(9600);
    for (int thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;  
}


void loop(){

    // Display Joystick values using the serial monitor
    outputJoystick();

    // Read the horizontal joystick value  (value between 0 and 1023)
    servoVal2 = analogRead(joyH);          
    servoVal2 = map(servoVal2, 0, 1023, 0, 180);     // scale it to use it with the servo (result  between 0 and 180)
    myservo2.write(averag(servoVal2));                         // sets the servo position according to the scaled value    

    // Read the horizontal joystick value  (value between 0 and 1023)
    servoVal1 = analogRead(joyV);           
    servoVal1 = map(servoVal1, 0, 1023, 0, 180);     // scale it to use it with the servo (result between 70 and 180)

    myservo1.write(averag(servoVal1));                           // sets the servo position according to the scaled value

    delay(15);                                       // waits for the servo to get there

}


/**
* Display joystick values
*/
void outputJoystick(){

    Serial.print(analogRead(joyH));
    Serial.print ("---"); 
    Serial.print(analogRead(joyV));
    Serial.println ("----------------");
}

int averag(int servoVal){
    // subtract the last reading:
  total= total - readings[index];        
  // read from the sensor:  
  readings[index] = servoVal;
  // add the reading to the total:
  total= total + readings[index];      
  // advance to the next position in the array:  
  index = index + 1;                    

  // if we're at the end of the array...
  if (index >= numReadings)              
    // ...wrap around to the beginning:
    index = 0;                          

  // calculate the average:
  average = total / numReadings;        
  // send it to the computer as ASCII digits
  return average;   }


