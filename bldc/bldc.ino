int Phase1[2]={2,6};
int Phase2[2]={3,7};
int Phase3[2]={4,8}; 

int incomingByte=0;
int D=5;

void setup() {             
  // initialize serial communication:
  Serial.begin(9600);
  pinMode(Phase1[0],OUTPUT);
  pinMode(Phase2[0],OUTPUT);
  pinMode(Phase3[0],OUTPUT);
  pinMode(Phase1[1],OUTPUT);
  pinMode(Phase2[1],OUTPUT);
  pinMode(Phase3[1],OUTPUT);

//pnp transistors -shwitch off
digitalWrite(Phase1[0], HIGH);
digitalWrite(Phase2[0], HIGH);
digitalWrite(Phase3[0], HIGH);

digitalWrite(Phase1[1], LOW);
digitalWrite(Phase2[1], LOW);
digitalWrite(Phase3[1], LOW);
}

// the loop routine runs over and over again forever:
void loop() {
  if (Serial.available() > 0) {
      // read the oldest byte in the serial buffer:
      incomingByte = Serial.read();
      // if it's a capital H (ASCII 72), turn on the LED:
      if (incomingByte == 'S') {
        Serial.print("S\r\n");
        delay(1000);
  
      }
      if (incomingByte == 'p'){
        Serial.print("D+\r\n");
        D+=1;
      }
      if (incomingByte == 'm'){
        Serial.print("D-\r\n");
        D-=1;
      }

      if(incomingByte == 'f'){
        Serial.print("f\r\n");
         //full
        nc(Phase3);
        plus(Phase1);
        minus(Phase2);
        //com2
        nc(Phase2);
        minus(Phase3);
        //com3
        nc(Phase1);
        plus(Phase2);
        //com4
        nc(Phase3);
        minus(Phase1);
        //com5
        nc(Phase2);
        plus(Phase3);
        //com6
        nc(Phase1);
        minus(Phase2);
        delay(D);
        
        }
  }



}

void plus(int Phase[]){
digitalWrite(Phase[1], LOW);
//delayMicroseconds(2);
digitalWrite(Phase[0], LOW);
}

void minus(int Phase[]){
 digitalWrite(Phase[1], HIGH);
 //delayMicroseconds(2);
digitalWrite(Phase[0], HIGH);
}

void nc(int Phase[]){
digitalWrite(Phase[0], HIGH);
//delayMicroseconds(2);
digitalWrite(Phase[1], LOW);
}


