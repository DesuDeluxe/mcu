#include <Arduino.h>

#include <avr/sleep.h>      // powerdown library
#include <avr/interrupt.h>  // interrupts library


#define DELAY 50


int interrupt = 2; // hardware interrupt, Motion Detector Open Collector pulls low
unsigned long awake_time = 0; // capture time when wake up from sleep
unsigned long elapsed_time = 0;  // time that have been awake for

//***************************************************
// *  Name:        pin2Interrupt, "ISR" to run when interrupted in Sleep Mode
void pin2Interrupt()
{
  /* This brings us back from sleep. */
}

//***************************************************
// *  Name:        enterSleep
void enterSleep()
{
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(0, pin2Interrupt, LOW);
  delay(50); // need this?
  /* the sleep modes
   SLEEP_MODE_IDLE - the least power savings
   SLEEP_MODE_ADC
   SLEEP_MODE_PWR_SAVE
   SLEEP_MODE_STANDBY
   SLEEP_MODE_PWR_DOWN - the most power savings
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // setting up for sleep ...
  sleep_enable();                       // setting up for sleep ...
  sleep_mode();                         // now goes to Sleep and waits for the interrupt

  /* The program will continue from here after the interrupt. */
  detachInterrupt(0);                 //disable interrupts while we wake up 

  /* First thing to do is disable sleep. */
  sleep_disable(); 

  // then go to the void Loop()
}

// ***********************************************************************
// set up the pins as Inputs, Outputs, etc.
void setup()
{

  pinMode (interrupt, INPUT);  // hardware interrupt for waking up
  digitalWrite (interrupt, HIGH);  // internal pullup enabled
 

awake_time = millis(); // capture wakeup time
//Serial.begin (19200); // for debug
delay (3000); // turn on, scan room
}

// ****************************************
// setup is done, start program
void loop()
{


// check if awake 10 seconds, then call sleep mode
elapsed_time = millis() - awake_time;  // see if 10 seconds gone by
//Serial.println (elapsed_time);
if (elapsed_time >=10000){                      // turn all light off
  delay (3000);                // time to walk away 
  enterSleep();               // call Sleep function to put us out
                              //  THE PROGRAM CONTINUES FROM HERE after waking up in enterSleep()
awake_time = millis();        // capture the time we woke up  
}
} // end void loop

