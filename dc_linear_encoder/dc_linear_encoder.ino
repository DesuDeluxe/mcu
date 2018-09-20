#include <Arduino.h>

/*
Arduino ink jet printer abuse
*/

#define EnkoderA 2                  //EnkoderA nam je spojen na pin 2
#define EnkoderB 3                  //EnkoderB nam je spojen na pin 3
#define Gas 9                      //Gas motora na pinu 9 
#define Ulaz1 4                    //Ulaz 1 motora na pinu 4 , sa ovim pinom cemo zajedno sa pinom 7 kontrolirati smijer
#define Ulaz2 7                    //Ulaz 2 motora na pinu 7

int trenutnaPozicija = 2000;       //nulta pozicija enkodera nije nula iz razloga da nam ne prelazi u negativnu vrijednost

void setup() {

    pinMode(EnkoderA, INPUT); //EnkoderA je ulaz
    digitalWrite(EnkoderA, HIGH); //pull up EncoderA
    pinMode(EnkoderB, INPUT); //EnkoderB je ulaz
    digitalWrite(EnkoderB, HIGH); //pull up EncoderB
    pinMode(Gas, OUTPUT); //Gas je izlaz
    analogWrite(Gas, 0); //Gas je 0 za pocetak (Speed PWM)
    pinMode(Ulaz1, OUTPUT); //Ulaz1 ce nam biti ulaz
    digitalWrite(Ulaz1, LOW); //Ulaz1 stavi na NISKO 
    pinMode(Ulaz2, OUTPUT); //Ulaz2 ce nam biti izlaz
    digitalWrite(Ulaz2, LOW); //Ulaz2 stavi na NISKO
    //Ako su Ulaz1 i Ulaz2 NISKO motor je iskljucen

    attachInterrupt(0, EnkoderOkidac, CHANGE); // Okidac koji se pokrece svaki puta kada se stanje na pinu 2 (interrupt 0)
                                               // promjeni iz NISKO u VISOKO ili obratno, znaci ako se desi promjena pokreni
                                               //funkciju pod nazivom EnkoderOkidac()

    // Otvori serijsku komunikaciju brzine 19200
    Serial.begin(19200);
     Serial.print("start ");
   }
 void loop() // run over and over
{ 
while(trenutnaPozicija<3000)  {   idiDesno();  analogWrite(Gas,255);    } ugasiMotor(); Serial.print("Stigli smo na 3000: "); Serial.println(trenutnaPozicija); delay(500); while(trenutnaPozicija>1000)
 { 
 idiLijevo();
 analogWrite(Gas,255); 
  }
ugasiMotor();
Serial.print("Stigli smo na 1000: ");
Serial.println(trenutnaPozicija);
delay(500);
}

void EnkoderOkidac() {

        detachInterrupt(0); // iskljuci inerrupt na pinu 2
    // Provjeri da li se vrijednost mijenja iz VISOKOG u NISKO ako da
    if (digitalRead(EnkoderB) == HIGH) {
        // Provjeri EnkoderA da saznaš u koju stranu se krecemo
        if (digitalRead(EnkoderA) == HIGH) {
            trenutnaPozicija++; // Desno dodaj 1

        } else {
            trenutnaPozicija--; // Lijevo
        }
    }// Ako se vrijednost mijenje iz NISKOG u VISOKO

    else {
        // Pogledaj EnkoderA da saznaš u koju stranu se krece
        if (digitalRead(EnkoderA) == LOW) {
            trenutnaPozicija++; // Desno dodaj 1            rpmcount++;
        }
        else {
            trenutnaPozicija--; // Desno dodaj 1
        }
    }
    attachInterrupt(0, EnkoderOkidac, CHANGE); // Ponovo ukljuci interrupt
}

void idiLijevo() {
    digitalWrite(Ulaz1, HIGH);
    digitalWrite(Ulaz2, LOW);//Podesi ulaze 1 i 2 da bi motor išao lijevo
}

void idiDesno() {
    digitalWrite(Ulaz1, LOW);
    digitalWrite(Ulaz2, HIGH);//Podesi ulaze 1 i 2 da bi motor išao desno
}

void ugasiMotor() {
    digitalWrite(Ulaz1, LOW);
    digitalWrite(Ulaz2, LOW); //Podesi ulaze 1 i 2 tako da je motor iskljucen
    analogWrite(Gas, 0); //Podesi Gas na 0
}


