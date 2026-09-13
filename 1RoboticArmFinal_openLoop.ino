//read from python code
String names;
unsigned int tym; // current time and previous time
char z;
int x = 0;
int y = 0;

//////////////////////
#include<Servo.h>
Servo s1;
Servo s2;
Servo s3;
Servo s4;
Servo s5;
Servo s6;
Servo s7;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Pins for Servo signal 2=s1 ..... 8=s7 ;s(n+1)=pins
  s1.attach(2);
  s2.attach(3);
  s3.attach(4);
  s4.attach(5);
  s5.attach(6);
  s6.attach(7);
  s7.attach(8);
  /////////////////////////////////////////////////
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //Pins for tracking the position of servos A0=s1 .... A6=s7; An-1=pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A6, INPUT);
  /////////////////// Time functionality
  tym = millis();
  delay(200);
  rst();


}

void loop() {
  // put your main code here, to run repeatedly:
  while ((millis() - tym) <= 5000) {
    if (Serial.available()) {
      names = Serial.readStringUntil('\r'); ////checking from python
    }
  }
tym=millis();
  if (names == "a") { //if bottle found
    pick_bottle();
tym=millis();
    while ((millis() - tym) <= 5000) {
      if (Serial.available()) {
        names = Serial.readStringUntil('\r'); //// check again
      }
    }
tym=millis();
    if (names == "c") { ////if only cup is found
      delay(2000);
      pick_cup();
    }
  }

  else if (names == "b") {
    pick_bottle();
tym=millis();
    while ((millis() - tym) <= 5000) {
      if (Serial.available()) {
        names = Serial.readStringUntil('\r'); //// check again
      }
    }
tym=millis();
    if (names == "c") { ////if only cup is found
      delay(2000);
      pick_cup();
    }
    else if (names == "b") { ////if only cup is found
      delay(2000);
      pick_bottle();
    }
  }
  else if (names == "c") {
    pick_cup();
tym=millis();
    while ((millis() - tym) <= 5000) {
      if (Serial.available()) {
        names = Serial.readStringUntil('\r'); //// check again
      }
    }
tym=millis();
    if (names == "b") { ////if only bottle is found
      delay(2000);
      pick_bottle();
    }
    else if (names == "c") { ////if only cup is found
      delay(2000);
      pick_cup();
    }


  }




}
void pick_bottle() { ///move 2,5,7

  if (x == 0) {
    rst();
    delay(600);
    x = 1;
  }

  //m1
  if (x == 1) {
    for (int i = 90; i <= 120; i++) {
      s1.write(i);
      delay(10);
    }
    x = 2;
  }

  //m5
  if (x == 2) {
    for (int i = 180; i >= 70; i--) {
      s5.write(i);
      delay(10);
    }
    x = 3;
  }

  //m7
  if (x == 3) {
    for (int i = 180; i >= 120; i--) {
      s7.write(i);
      delay(10);
    }
    x = 4;
  }

  //m2
  if (x == 4) {
    for (int i = 30; i <= 150; i++) {
      s2.write(i);
      s3.write(180 - i);
      delay(10);
    }
    x = 5;
  }

  //m7
  if (x == 5) {
    for (int i = 120; i <= 170; i++) {
      s7.write(i);
      delay(10);
    }
    x = 6;
  }

  //m2
  if (x == 6) {
    for (int i = 150; i >= 90; i--) {
      s2.write(i);
      s3.write(180 - i);
      delay(10);
    }
    x = 7;
  }

  //m1
  if (x == 7) {
    for (int i = 120; i <= 180; i++) {
      s1.write(i);
      delay(10);
    }
    x = 8;
  }

  //m2
  if (x == 8) {
    for (int i = 90; i <= 150; i++) {
      s2.write(i);
      s3.write(180 - i);
      delay(10);
    }
    x = 9;
  }

  //m7
  if (x == 9) {
    for (int i = 170; i >= 120; i--) {
      s7.write(i);
      delay(10);
    }
    x = 10;
  }

  if (x == 10) {
    rst();
    x = 11;
  }





}


void pick_cup() { ///move 2,5,7

  if (y == 0) {
    rst();
    delay(600);
    y = 1;
  }

  //m1
  if (y == 1) {
    for (int i = 90; i >= 45; i--) {
      s1.write(i);
      delay(10);
    }
    y = 2;
  }

  //m5
  if (y == 2) {
    for (int i = 180; i >= 70; i--) {
      s5.write(i);
      delay(10);
    }
    y = 3;
  }

  //m7
  if (y == 3) {
    for (int i = 180; i >= 120; i--) {
      s7.write(i);
      delay(10);
    }
    y = 4;
  }

  //m2
  if (y == 4) {
    for (int i = 30; i <= 150; i++) {
      s2.write(i);
      s3.write(180 - i);
      delay(10);
    }
    y = 5;
  }

  //m7
  if (y == 5) {
    for (int i = 120; i <= 170; i++) {
      s7.write(i);
      delay(10);
    }
    y = 6;
  }

  //m2
  if (y == 6) {
    for (int i = 150; i >= 90; i--) {
      s2.write(i);
      s3.write(180 - i);
      delay(10);
    }
    y = 7;
  }

  //m1
  if (y == 7) {
    for (int i = 45; i >= 0; i--) {
      s1.write(i);
      delay(10);
    }
    y = 8;
  }

  //m2
  if (y == 8) {
    for (int i = 90; i <= 150; i++) {
      s2.write(i);
      s3.write(180 - i);
      delay(10);
    }
    y = 9;
  }

  //m7
  if (y == 9) {
    for (int i = 170; i >= 120; i--) {
      s7.write(i);
      delay(10);
    }
    y = 10;
  }

  if (y == 10) {
    rst();
    y = 11;
  }





}

void rst() {
  s1.write(90);
  s2.write(30);
  s3.write(150);
  s4.write(90);
  s5.write(180);
  s6.write(0);
  s7.write(180);
}
