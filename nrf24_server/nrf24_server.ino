// nrf24_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_NRF24 class. RH_NRF24 class does not provide for addressing or
// reliability, so you should only use RH_NRF24  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example nrf24_client
// Tested on Uno with Sparkfun NRF25L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <SPI.h>
#include <RH_NRF24.h>

// Singleton instance of the radio driver
 RH_NRF24 nrf24(8, 10);
//#define AlarmPin
#define DoorPin 1
#define Window1Pin 2
#define Window2Pin 3
#define Window3Pin 4
#define Window4Pin 5
#define PirPin 6
#define WaterPin 7
#define LightPin 8
#define LightPin2 9

static String reading[10];
static String switching[3];
uint8_t repl[4]="Nok"; //helper variablr for function return
void setup() 
{

//    pinMode(1,INPUT);
//    pinMode(2,INPUT);
//    pinMode(3,INPUT);
//    pinMode(4,INPUT);
//    pinMode(5,INPUT);
//    pinMode(6,INPUT);
//    pinMode(7,INPUT);
//    pinMode(8,INPUT);
//    pinMode(9,INPUT);
//    
//    pinMode(10,OUTPUT);
//    pinMode(11,OUTPUT);
//    pinMode(12,OUTPUT);

//##################################################################
//testing without connecting anything
randomSeed(analogRead(0));

    reading[0]="All";
    reading[1]=DoorPin;
    reading[2]=Window1Pin;
    reading[3]=Window2Pin;
    reading[4]=Window3Pin;
    reading[5]=Window4Pin;
    reading[6]=PirPin;
    reading[7]=WaterPin;
    reading[8]=LightPin;
    reading[9]=LightPin2;

    switching[0]=WaterPin;
    switching[1]=LightPin;
    switching[2]=LightPin2;
    

  Serial.begin(9600);
  while (!Serial) 
    ; // wait for serial port to connect
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");    
 Serial.println("Server start");    
}

void loop()
{
  if (nrf24.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
//      RF24::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println((uint8_t)rf24.lastRssi(), DEC);

    int ch=*buf-'0';
   if(ch==67){//corresponds to "s" sent from client
    while(!nrf24.available())
    {}
       if (nrf24.recv(buf, &len))
       { 
         change_status(*buf);
         uint8_t data[4] ={*repl};
         nrf24.send(repl, sizeof(repl));
         nrf24.waitPacketSent();
         Serial.println("Sent a reply(result) ");
       }
    //}
    
  }
  else{
         read_status(*buf);
         uint8_t data[4] ={*repl};
         nrf24.send(repl, sizeof(repl));
         nrf24.waitPacketSent();
         Serial.println("Sent a reply(status) ");
  }
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}


//String read_status(int sens){
//  if(digitalRead(reading[(sens-'0')]) == 1) return "On"
//  else return "Off"
//}
//
//
//char change_status(int sens){
//  digitalWrite(switching[(sens-'0')]);
//  return "k"
//}
//##################################################################
//testing without connecting anything


void read_status(uint8_t sens){
  int randomnumber = random(1);
//Serial.print(sens- '0');
  String str;
  if(randomnumber == 0){
    str="On ";
    }
  else {
    str="Off";
  }
  str.getBytes(repl, 4);
}


void change_status(uint8_t sens){
  String str="is ";
  str.getBytes(repl, 4);
}
