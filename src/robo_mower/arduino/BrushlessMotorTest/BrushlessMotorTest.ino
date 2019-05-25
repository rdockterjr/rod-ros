
#include <ESC.h> //used for esc control

const int right_pin = 23;
ESC ESC_Right(right_pin);

int control_out = 0;

void setup()
{
   ESC_Right.Init() ;  //Set pin 2 as output
   Serial.begin(9600);
}
void loop()
{
   if (Serial.available()){
    control_out = Serial.parseInt();
    while(Serial.available()){
      byte who = Serial.read();
    }
    Serial.print("I received :");
    Serial.println(control_out);
   }
   ESC_Right.Control(control_out) ;     //setting pwm 
   Serial.print("Servo Val: ");
   Serial.println(ESC_Right._lastpwm);
   delay(50) ;
}
