#include <Arduino.h>

void setup(){
Serial.begin(9600);

Serial.println("AT+CMGF=0"); //puts the phone in pdu mode

delay(3000); //processing time

Serial.println("atd123456789;");
delay(10000);
}


void loop(){
 
//Wysłanie SMS'a
Serial.println("AT+CMGF=1"); // Przejście w tryb SMS
delay(1000);
Serial.println("AT+CMGS=\"65856568\""); // Tworzy nowego SMS na podany numer
delay(1000);
Serial.print("wiadomosc"); // Zawartość SMS'a
delay(1000);
Serial.write(byte(26)); // Sygnał końca wiadomości
delay(10000); // Czas dla telefonu na wysłanie wiadomości SMS
}


