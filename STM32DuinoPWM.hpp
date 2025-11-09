#include "HardwareTimer.h"
#include "stm32g474xx.h"
#include <Arduino.h>

enum PwmRange {
  LOWFREQ, 
  HIGHFREQ,
};

struct PinTimerMap {
    uint8_t pin;
    TIM_TypeDef *timer;
    uint8_t channel;
};

extern PinTimerMap timerMap[];

TIM_TypeDef *getTimerForPin(uint8_t pin);
int getChannelForPin(uint8_t pin);

/*  ---------------------------  *
 *  -- BASE TIMER CLASS      --  *
 *  ---------------------------  */
class STM32HALPWM {
  protected:
    HardwareTimer *halTimer;
    uint8_t pwmPin;
    uint8_t channel;
    bool initialized;

  public:
    STM32HALPWM(uint8_t pin);
    virtual ~STM32HALPWM();

    virtual void begin() = 0;
    virtual void   end() = 0;

    TIM_TypeDef *getTimerInstance() const;
    uint8_t getChannel() const;

};

/*  ---------------------------  *
 *  -- PWM GENERATOR CLASS   --  *
 *  ---------------------------  */
class OutputPWM : public STM32HALPWM {
  private:
    uint32_t frequency;
    float dutyCycle;

  public:
    OutputPWM(uint8_t pin, uint32_t freq = 128, float duty = 50.0f);

    void begin() override;
    void   end() override;

    void setFrequency(uint32_t freq);
    void setDutyCycle(float duty);
    void enable();
    void disable();
};

/*  ---------------------------  *
 *  -- PWM MEASUREMENT CLASS --  *
 *  ---------------------------  */
class InputPWM : public STM32HALPWM {
  private:
    volatile uint32_t lastRising;
    volatile uint32_t lastFalling;
    volatile uint32_t period;
    volatile uint32_t pulseWidth;

    uint16_t prescalerValue; 
    static void captureCallback(void *arg);

  public:
    InputPWM(uint8_t pin, PwmRange range=LOWFREQ);
    void handleCapture(); 

    void begin() override;
    void   end() override;

    float getFrequency();
    float getDutyCycle();
};
