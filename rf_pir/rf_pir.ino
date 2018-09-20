#include <Arduino.h>

//simple Tx on pin D12
//Written By : Mohannad Rawashdeh
// 3:00pm , 13/6/2013
//http://www.genotronex.com/
//..................................
#include <VirtualWire.h>
#define PIR 11
char *controller;
void setup() {
  pinMode(13,OUTPUT);
// pinMode(10,INPUT_PULLUP);
pinMode(PIR,INPUT);
vw_set_ptt_inverted(true); //
vw_set_tx_pin(12);
vw_setup(4000);// speed of data transfer Kbps
}

void loop(){

  if(digitalRead(PIR)==HIGH){
controller="1"  ;
vw_send((uint8_t *)controller, strlen(controller));
vw_wait_tx(); // Wait until the whole message is gone
digitalWrite(13,1);

delay(2000);
controller="0"  ;
vw_send((uint8_t *)controller, strlen(controller));
vw_wait_tx(); // Wait until the whole message is gone
  }
 else digitalWrite(13,0);
{delay(2000);}

}
