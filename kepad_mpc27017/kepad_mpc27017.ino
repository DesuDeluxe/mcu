#include <Arduino.h>

/* @file MultiKey.ino
|| @version 1.0
|| @author Mark Stanley
|| @contact mstanley@technologist.com
||
|| @description
|| | The latest version, 3.0, of the keypad library supports up to 10
|| | active keys all being pressed at the same time. This sketch is an
|| | example of how you can get multiple key presses from a keypad or
|| | keyboard.
|| #
*/

#include <Keypad.h>
#include <Wire.h>

const byte ROWS = 8; //four rows
const byte COLS = 8; //three columns
char keys[ROWS][COLS] = {
{'0','1','2','3','4','5','6','7'},
{'01','11','21','31','41','51','61','71'},
{'02','12','22','32','42','52','62','72'},
{'03','13','23','33','43','53','63','73'},
{'04','14','24','34','44','54','64','74'},
{'05','15','25','35','45','55','65','75'},
{'06','16','26','36','46','56','66','76'},
{'07','17','27','37','47','57','67','77'}
};
byte rowPins[ROWS] = {0,1,2,3,4,5,6,7};

byte colPins[COLS] = {15,14,13,12,11,10,9,8}; 


Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

unsigned long loopCount;
unsigned long startTime;
String msg;


void setup() {
 mcp.begin();
  
    Serial.begin(9600);
    loopCount = 0;
    startTime = millis();
    msg = "";
    Serial.print("start");
}


void loop() {
    loopCount++;
    if ( (millis()-startTime)>5000 ) {
        Serial.print("Average loops per second = ");
        Serial.println(loopCount/5);
        startTime = millis();
        loopCount = 0;
    }

    // Fills kpd.key[ ] array with up-to 10 active keys.
    // Returns true if there are ANY active keys.
//    if (kpd.getKeys())
    {
        for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
        {
            if ( kpd.key[i].stateChanged )   // Only find keys that have changed state.
            {
                switch (kpd.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
                    case PRESSED:
                    msg = " PRESSED.";
                break;
                    case HOLD:
                    msg = " HOLD.";
                break;
                    case RELEASED:
                    msg = " RELEASED.";
                break;
                    case IDLE:
                    msg = " IDLE.";
                }
                Serial.print("Key ");
                Serial.print(kpd.key[i].kchar);
                Serial.println(msg);
            }
        }
    }
}  // End loop
