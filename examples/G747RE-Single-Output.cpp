#include "STM32DuinoPWM.hpp"

OutputPWM pwmOut(A5); 

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting up...");

  Serial.print("Starting PWM Output : ");
  pwmOut.begin();
  Serial.println("Done!");
}

void loop() {
  pwmOut.setFrequency(1000);
  pwmOut.setDutyCycle(85); 

  Serial.print("Frequency  : ");
  Serial.println(pwmOut.getFrequency()); 
  Serial.print("Duty Cycle : ");
  Serial.println(pwmOut.getDutyCycle());
  Serial.println();

  delay(1000);

  pwmOut.setFrequency(2000);
  pwmOut.setDutyCycle(15);

  Serial.print("Frequency  : ");
  Serial.println(pwmOut.getFrequency()); 
  Serial.print("Duty Cycle : ");
  Serial.println(pwmOut.getDutyCycle());
  Serial.println();

  delay(1000);
}
