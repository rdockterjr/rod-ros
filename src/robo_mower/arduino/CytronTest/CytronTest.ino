#include <Cytron.h>

//https://www.cytron.io/c-93-dc-motor-driver/dc-motor-driver/p-md30c
//https://docs.google.com/document/d/178uDa3dmoG0ZX859rWUOS2Xyafkd8hSsSET5-ZLXMYQ/view

int pwmpin1 = 11;
int dirpin1 = 12;

//cytron class
Cytron cytron(pwmpin1, dirpin1, HIGH);

void setup() {
  // set up pin modes
  cytron.Init();

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // read the val
    int percent = Serial.parseInt();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(percent);

    cytron.Control(percent);
  }
  

}
