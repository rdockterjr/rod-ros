
#include <ESC.h> //used for esc control

const int right_pin = 23;
const int left_pin = 21;
ESC ESC_Right(right_pin);
ESC ESC_Left(left_pin);

int control_out = 0;

void setup()
{
   ESC_Right.Init() ;
   ESC_Left.Init() ;
   Serial.begin(9600);

   //Doesnt matter
  //analogWriteResolution(10); //0-1023 range
  //analogWriteFrequency(35, 58593);
}
void loop()
{
   if (Serial.available()){
    control_out = Serial.parseInt();
    while(Serial.available()){
      byte who = Serial.read();
    }
    Serial.print("I received: ");
    Serial.println(control_out);
   }
   ESC_Right.Control(control_out) ;     //setting pwm 
   ESC_Left.Control(control_out);
   
   //Serial.print("Servo Val: ");
   //Serial.println(ESC_Right._lastpwm);
   delay(50) ;
}
