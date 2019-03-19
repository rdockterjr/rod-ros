#include <Cytron.h>

//https://www.cytron.io/c-93-dc-motor-driver/dc-motor-driver/p-md30c
//https://docs.google.com/document/d/178uDa3dmoG0ZX859rWUOS2Xyafkd8hSsSET5-ZLXMYQ/view

int pwmpinl = 38;
int dirpinl = 39;
int pwmpinr = 35;
int dirpinr = 34;

//cytron class
Cytron left_motor(pwmpinl, dirpinl, HIGH);
Cytron right_motor(pwmpinr, dirpinr, HIGH);

void setup() {
  // set up pin modes
  left_motor.Init();
  right_motor.Init();

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    // read the val
    int percent = Serial.parseInt();
    while(Serial.available() >0){
      Serial.read(); //ignore
    }

    // say what you got:
    Serial.print("pwm: ");
    Serial.println(percent);

    left_motor.Control(percent);
    right_motor.Control(percent);
  }
  delay(100);

}
