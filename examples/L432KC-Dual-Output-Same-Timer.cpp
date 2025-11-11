/*  -------------------------------------------------  *
 *  -- A1 and A4 are both tied to hardware timer 2 --  *
 *  -- hardware timers can only oscillate at one   --  *
 *  -- frequency, but each output can have it's    --  *
 *  -- own individual duty cycle. changing the     --  *
 *  -- frequency for either output object would    --  *
 *  -- change the frequency for both.              --  *
 *  -------------------------------------------------  */


#include "STM32DuinoPWM.hpp"

OutputPWM pwmOut1(A1); 
OutputPWM pwmOut2(A4); 

float dc1 = 90;
float dc2 = 10;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting up...");

  Serial.print("Starting PWM Output 1 : ");
  pwmOut1.begin();
  Serial.println("Done!");
  
  Serial.print("Starting PWM Output 2 : ");
  pwmOut2.begin();
  Serial.println("Done!");

  pwmOut1.setFrequency(1000);
  pwmOut1.setDutyCycle(dc1);
  pwmOut2.setDutyCycle(dc2);
}

void loop() {
  if (dc1 == 10)
    dc1 = 90; 
  else 
    --dc1;

  if (dc2 == 90)
    dc2 = 10; 
  else
   ++dc2;

  pwmOut1.setDutyCycle(dc1);
  pwmOut2.setDutyCycle(dc2);

  delay(100);
}
