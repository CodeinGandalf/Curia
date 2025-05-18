#include <Servo.h>

Servo lift;
Servo myservo;

//Servo nicht einfach so auseinandersvhrauben!!! (ansonsten Ryoya Bauer fragen)

int unten = 2325;
int oben = 1100;

int mitte = 1500;
int geschlossen = 1080;
int offen = 1650;


void setup() {
  lift.attach(10);                 // Servo an Pin 10
  lift.writeMicroseconds(oben);  // Anschlag ganz unten

  myservo.attach(9);                 // Servo an Pin 9
  myservo.writeMicroseconds(offen);  // Anschlag ganz links

  delay(1500);
}

void loop() {

  //Servo fährt herunter
  for (int pulseWidth = oben; pulseWidth <= unten; pulseWidth += 10) {
    lift.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1500);

  // Offen (1750 µs) bis Greiffer zu (1080 us)
  for (int pulseWidth = offen; pulseWidth >= geschlossen; pulseWidth -= 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);


  //Servo fährt hoch (ca. 3cm oder 120 Grad)
  for (int pulseWidth = unten; pulseWidth >= oben; pulseWidth -= 10) {
    lift.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1500);


  //Servo fährt herunter
  for (int pulseWidth = oben; pulseWidth <= unten; pulseWidth += 10) {
    lift.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1500);

  // Greifer öffnen (bis 1750 us)
  for (int pulseWidth = geschlossen; pulseWidth <= offen; pulseWidth += 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);

  //Servo fährt hoch (ca. 3cm oder 120 Grad)
  for (int pulseWidth = unten; pulseWidth >= oben; pulseWidth -= 10) {
    lift.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1500);

}