#include <Servo.h>

Servo myservo;

//Servo nicht einfach so auseinandersvhrauben!!! (ansonsten Ryoya Bauer fragen)

void setup() {
  myservo.attach(9);                 // Servo an Pin 9
  myservo.writeMicroseconds(500);  // Anschlag ganz links
  delay(1000);
}

void loop() {
  // Von Mitte (1500 µs) bis Greiffer zu (1080 us)
  for (int pulseWidth = 1500; pulseWidth >= 1080; pulseWidth -= 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);

  // Greifer öffnen (bis 1750 us)
  for (int pulseWidth = 1080; pulseWidth <= 1750; pulseWidth += 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);


  //Zurück in die Mitte
  for (int pulseWidth = 1750; pulseWidth >= 1500; pulseWidth -= 10) {
    myservo.writeMicroseconds(pulseWidth);
    delay(15);
  }

  delay(1000);

}
