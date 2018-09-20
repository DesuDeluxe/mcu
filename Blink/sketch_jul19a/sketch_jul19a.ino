#include <Arduino.h>


int alarm=5;
int led1=13;
void setup() {
  // put your setup code here, to run once:
  pinMode(led1, OUTPUT);
  pinMode(alarm, INPUT_PULLUP);
   delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
 if (digitalRead(alarm)==LOW){
            digitalWrite(led1,LOW);
            delay(2000);
        }
   else{
           digitalWrite(led1,HIGH);
           }
}

