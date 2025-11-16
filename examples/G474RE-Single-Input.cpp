#include "STM32DuinoPWM.hpp"

InputPWM pwmIn(A5, LOWFREQ);

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Starting up...");

    Serial.print("Staring PWM Input : "); 
    pwmIn.begin();
    Serial.println("Done!");
}

void loop() {
    float frequency = pwmIn.getFrequency(); 
    float dutyCycle = pwmIn.getDutyCycle();

    Serial.print("Frequency  : ");
    Serial.println(frequency); 
    Serial.print("Duty Cycle : ");
    Serial.println(dutyCycle); 
    Serial.println();

    delay(1000);
}
