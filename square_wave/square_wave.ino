//simple 500Hz square wave
//by Amanda Ghassaei 2012

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

//sends 500Hz pulse wave out arduino D8

void setup() {
  //set digital pin 8 as output
  pinMode(8,OUTPUT);
}

void loop(){
  digitalWrite(8,HIGH);//set pin 8 to +5V
  delay(1);//wait 1ms
  digitalWrite(8,LOW);//set pin 8 to 0V
  delay(1);//wait 1ms
}

