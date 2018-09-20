#include <Arduino.h>

#include <SoftwareSerial.h>
 
SoftwareSerial mySerial(3, 2); // RX, TX pins
 
void setup() {
 
pinMode(13, OUTPUT); // Initialize pin 13 as digital out (LED)

mySerial.begin(9600); // Open serial connection at baud rate of 4800
 
}
 
void loop(){
 

digitalWrite(13, HIGH); // Turn LED on.
mySerial.println("AT"); // Sends AT command to wake up cell phone
delay(500);
mySerial.println("AT+CMGF=1"); // Puts phone into SMS mode
delay(1000); // Wait a second
mySerial.println("AT+CMGW=\"+48733944541\""); // YOUR NUMBER HERE; Creates new message to number
delay(1000);
mySerial.print("Sent from my Arduino."); // Message contents
delay(1000);
mySerial.write(byte(26)); // (signals end of message)
delay(1000);
mySerial.println("AT+CMSS=1"); // Sends message at index of 1
digitalWrite(13, LOW); // Turn LED off
delay(250);
digitalWrite(13, HIGH); // Turn LED on.
delay(10000); // Give the phone time to send the SMS
mySerial.println("AT+CMGD=1"); // Deletes message at index of 1
digitalWrite(13, LOW); // Turn LED off.
delay(2000);

 
}
