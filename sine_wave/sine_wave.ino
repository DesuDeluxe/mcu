/*
 Copyright 2007 Richard Cappels used with his permission
 www.projects.cappels.org
 This assumes an R/2R DAC (or other DAC) connected to PORTD, and
 Arduino 16mhz clock
 The frequency measured at the output of the DAC is 1.007 kHz.
 ported to arduino by hotcarrier
 output to pd7-pd2 with 6 bits- needs high order pd7 to work
 correctly
 */

#include <avr/interrupt.h>
#include <stdlib.h>

char sinetable [32];
int  i ;

void ioinit (void)
{
  //Initialize output ports
  PORTD = B11111111;
  DDRD  = B11111111;

}

void timer_setup(){
  TCCR2A = 0;
  TCNT2=455;    //455 outputs 1.007khz
  TCCR2B = B00000010;
  //Timer2 Overflow Interrupt Enable
  TIMSK2 = 1<<TOIE2;
}
void setup(){           

  ioinit();
  arraysetup();
  cli();
  timer_setup();
  i = 0;
  sei();

}


ISR(TIMER2_OVF_vect) {

  PORTD=(sinetable[i++]);
  TCNT2=455;
  if(i==32){
    i=0;
  }

}
void arraysetup(void){
  sinetable[0]=127;  // Put 32 step 8 bit sine table into array.
  sinetable[1]=152;
  sinetable[2]=176;
  sinetable[3]=198;
  sinetable[4]=217;
  sinetable[5]=233;
  sinetable[6]=245;
  sinetable[7]=252;
  sinetable[8]=254;
  sinetable[9]=252;
  sinetable[10]=245;
  sinetable[11]=233;
  sinetable[12]=217;
  sinetable[13]=198;
  sinetable[14]=176;
  sinetable[15]=152;
  sinetable[16]=128;
  sinetable[17]=103;
  sinetable[18]=79;
  sinetable[19]=57;
  sinetable[20]=38;
  sinetable[21]=22;
  sinetable[22]=10;
  sinetable[23]=3;
  sinetable[24]=0;
  sinetable[25]=3;
  sinetable[26]=10;
  sinetable[27]=22;
  sinetable[28]=38;
  sinetable[29]=57;
  sinetable[30]=79;
  sinetable[31]=103;
}
void loop()
{

  while (1)
  {

  }

}
