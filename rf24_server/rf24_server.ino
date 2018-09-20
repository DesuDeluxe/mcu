// rf24_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_RF24 class. RH_RF24 class does not provide for addressing or
// reliability, so you should only use RH_RF24  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf24_client
// Tested on Anarduino Mini http://www.anarduino.com/mini/ with RFM24W and RFM26W

#include <SPI.h>
#include <RH_RF24.h>

#define DoorPin 1
#define Window1Pin 2
#define Window2Pin 3
#define Window3Pin 4
#define Window4Pin 5
#define PirPin 6
#define WaterPin 7
#define LightPin 8
#define LightPin2 9

// Singleton instance of the radio driver
RH_RF24 rf24;

static String reading[10];
static String switching[3];
unsigned char* repl;

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

    
  Serial.begin(9600);
  if (!rf24.init())
    Serial.println("init failed");
  // The default radio config is for 30MHz Xtal, 434MHz base freq 2GFSK 5kbps 10kHz deviation
  // power setting 0x10
  // If you want a different frequency mand or modulation scheme, you must generate a new
  // radio config file as per the RH_RF24 module documentation and recompile
  // You can change a few other things programatically:
  //rf24.setFrequency(435.0); // Only within the same frequency band
  //rf24.setTxPower(0x7f);
  
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


  
}

void loop()
{
  if (rf24.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf24.recv(buf, &len))
    {
//      RF24::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println((uint8_t)rf24.lastRssi(), DEC);


  if ((char*)buf == "s"){
    if (rf24.available())
    {
       if (rf24.recv(buf, &len))
       { 
         repl=change_status(buf).
         //repl.toCharArray(buf, len) ;
         rf24.send(repl, sizeof(repl));
         rf24.waitPacketSent();
         Serial.println("Sent a reply");
       }
    }
    
  }
  else{
         repl = read_status(buf);
         repl.toCharArray(buf, len) ;
         rf24.send(buf, sizeof(buf));
         rf24.waitPacketSent();
         Serial.println("Sent a reply");
  }



      rf24.send(data, sizeof(data));
      rf24.waitPacketSent();
      Serial.println("Sent a reply");
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}


//String read_status(int sens){
//  if(digitalRead(reading[sens]) == 1) return "On"
//  else return "Off"
//}
//
//
//char change_status(int sens){
//  digitalWrite(switching[sens]);
//  return "k"
//}



//##################################################################
//testing without connecting anything

unsigned char* read_status(uint8_t* sens){
  randomnumber = random(1);
  if(randomnumber == 1){ return "On";}
  else {
  return "Off";}
}


unsigned char* change_status(uint8_t* sens){
  Serial.println("Switched pin: " + switching[sens]);
  //digitalWrite(switching[sens]);
  return "k";
}
