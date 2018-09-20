#include <Arduino.h>

/* Analog Read Interrupter
* -----------------------
*/

int ruptPin = 2; // select the input pin for the interrupter
int val = 0; // variable to store the value coming from the sensor

int ruptPin2 = 3; // select the input pin for the interrupter
int val2 = 0; // variable to store the value coming from the sensor

void setup()
{
    Serial.begin(9600); // set up Serial library at 9600 bps
}

void loop()
{
    val = analogRead(ruptPin); // read the value from the sensor
    Serial.println(val); // print the sensor value to the serial monitor
        val2 = analogRead(ruptPin2); // read the value from the sensor
    Serial.println(val2); // print the sensor value to the serial monitor
    delay(50);
    delay(50);
}
