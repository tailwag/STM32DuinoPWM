#include "STM32DuinoPWM.hpp"

OutputPWM PwmOut(PC0, 10000, 50.0f); 
InputPWM  PwmIn(PA0, LOWFREQ); 

void setup() {
  Serial.begin(115200);
  HAL_Delay(500);
  Serial.println("starting up");

  PwmOut.begin();
  Serial.println("output initialized");
  PwmIn.begin();
  Serial.println("input initialized");

  PwmOut.setFrequency(128);
  PwmOut.setDutyCycle(40.0);
  PwmOut.enable();
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Measured freq: ");
    Serial.print(PwmIn.getFrequency());
    Serial.print(" Hz, duty: ");
    Serial.print(PwmIn.getDutyCycle());
    Serial.println(" %");
  }
}
