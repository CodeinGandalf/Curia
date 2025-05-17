#include <Servo.h>

Servo myservo;

//Servo nicht einfach so auseinandersvhrauben!!! (ansonsten Ryoya Bauer fragen)

int unten = 2000;
int oben = 800;

void setup() {
  myservo.attach(10);                 // Servo an Pin 9
  myservo.writeMicroseconds(unten);  // Anschlag ganz links
  delay(1000);
}

void loop() {


  //Servo fährt hoch (ca. 3cm oder 120 Grad)
  for (int pulseWidth = unten; pulseWidth >= oben; pulseWidth -= 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);

  //Servo fährt herunter
  for (int pulseWidth = oben; pulseWidth <= unten; pulseWidth += 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);

}
