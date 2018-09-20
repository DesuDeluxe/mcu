#include <Arduino.h>

#include <VirtualWire.h>
 
 
const char *wiadomosc;
 
void setup() {
  vw_set_ptt_inverted(true);
  vw_setup(2000);
 wiadomosc = "listy";
}
 
void loop() {
 
  vw_send((uint8_t *)wiadomosc, strlen(wiadomosc));
  vw_wait_tx();
}
