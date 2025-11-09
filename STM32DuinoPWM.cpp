#include "STM32DuinoPWM.hpp"
#include "WSerial.h"
#include <cstdint>

static InputPWM *inputInstances[32] = {nullptr};

template <int8_t N>
void captureWrapper() {
  if (inputInstances[N])
    inputInstances[N]->handleCapture();
}

using CaptureFunc = void(*)(); 

static const CaptureFunc captureWrappers[32] = {
  captureWrapper<0>, captureWrapper<1>, captureWrapper<2>, captureWrapper<3>, captureWrapper<4>, 
  captureWrapper<5>, captureWrapper<6>, captureWrapper<7>, captureWrapper<8>, captureWrapper<9>,
  captureWrapper<10>, captureWrapper<11>, captureWrapper<12>, captureWrapper<13>, captureWrapper<14>,
  captureWrapper<15>, captureWrapper<16>, captureWrapper<17>, captureWrapper<18>, captureWrapper<19>,
  captureWrapper<20>, captureWrapper<21>, captureWrapper<22>, captureWrapper<23>, captureWrapper<24>,
  captureWrapper<25>, captureWrapper<26>, captureWrapper<27>, captureWrapper<28>, captureWrapper<29>,
  captureWrapper<30>, captureWrapper<31>
};

PinTimerMap timerMap[] = {
  //#ifdef F401RE
  //  {PA8,  TIM1, 1}, //D7 
  //  {PA9,  TIM1, 2}, //D8
  //  {PA10, TIM1, 3}, //D2
  //  {PA11, TIM1, 4},
  //  {PA0,  TIM2, 1}, //A0
  //  {PA1,  TIM2, 2}, //A1
  //  {PB10, TIM2, 3}, //D6
  //  {PA6,  TIM3, 1}, //D12
  //  {PA7,  TIM3, 2}, //D11
  //  {PB0,  TIM3, 3}, //A3
  //  {PB1,  TIM3, 4}, 
  //  {PB6,  TIM4, 1}, //D10
  //  {PB7,  TIM4, 2},
  //  {PB8,  TIM4, 3}, //D15
  //  {PB9,  TIM4, 4}, //D14
  //#endif
  #ifdef G474RE
    {PC0,  TIM1, 1}, // L38, A5
    {PC1,  TIM1, 2}, // L36, A4
    {PC2,  TIM1, 3}, // L35
    {PC3,  TIM1, 4}, // L37
    {PA0,  TIM2, 1}, // L28, A0
    {PA1,  TIM2, 2}, // L30, A1
    {PB10, TIM2, 3}, // R25, D6
    {PB11, TIM2, 4}, // R18
    {PA6,  TIM3, 1}, // R13, D12
    {PA4,  TIM3, 2}, // L32, A2
    {PB0,  TIM3, 3}, // L34, A3
    {PB1,  TIM3, 4}, // R24 
    {PB6,  TIM4, 1}, // R17, D10
    {PB7,  TIM4, 2}, // L21
    {PB8,  TIM4, 3}, // R03, D15
    {PB9,  TIM4, 4}, // R05, D14
  #endif
};
TIM_TypeDef * getTimerForPin(uint8_t pin) {
    for (auto &m : timerMap) {
        if (m.pin == pin) return m.timer; 
    }
    return nullptr; // not found
}

int8_t getChannelForPin(uint8_t pin) {
    for (auto &m : timerMap) {
        if (m.pin == pin) return m.channel; 
    }
    return -1;
}

int8_t getOutputIdForPin(uint8_t pin) {
  int8_t id = 0;
  for (auto &m : timerMap) {
    if (m.pin == pin) return id;
    ++id;
  }
  return -1;
}

/*  ---------------------------  *
 *  -- BASE TIMER CLASS      --  *
 *  ---------------------------  */
STM32HALPWM::STM32HALPWM(uint8_t pin) 
  : halTimer(nullptr), pwmPin(pin), channel(0), initialized(false) {

  TIM_TypeDef *timer = getTimerForPin(pin);
  channel  = getChannelForPin(pin);
  outputId = getOutputIdForPin(pin);
  if (timer)
    halTimer = new HardwareTimer(timer);
}

STM32HALPWM::~STM32HALPWM() {
  if (halTimer) {
    delete halTimer; 
    halTimer = nullptr;
  }
}

TIM_TypeDef *STM32HALPWM::getTimerInstance() const {
  return halTimer ? halTimer->getHandle()->Instance : nullptr;
}


/*  ---------------------------  *
 *  -- PWM GENERATOR CLASS   --  *
 *  ---------------------------  */
OutputPWM::OutputPWM(uint8_t pin, uint32_t freq, float dutyCycle)
  : STM32HALPWM(pin), frequency(freq), dutyCycle(dutyCycle) {}

void OutputPWM::begin() {
  halTimer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pwmPin);
  halTimer->setOverflow(frequency, HERTZ_FORMAT);
  halTimer->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
  halTimer->resume(); 
  initialized = true;
}

void OutputPWM::end() {
  if (halTimer) halTimer->pause();
  initialized = false;
}

void OutputPWM::setFrequency(uint32_t freq) {
  frequency = freq;
  if (initialized)
    halTimer->setOverflow(frequency, HERTZ_FORMAT);
}

void OutputPWM::setDutyCycle(float duty) {
  dutyCycle = constrain(duty, 0.0f, 100.0f);
  if (initialized) {
    uint32_t arr = halTimer->getOverflow();
    uint32_t ccr = (arr * dutyCycle) / 100.0f;
    halTimer->setCaptureCompare(channel, ccr, TICK_COMPARE_FORMAT);
  }
}

void OutputPWM::enable() {
  if (halTimer) halTimer->resume();
}

void OutputPWM::disable() {
  if (halTimer) halTimer->pause();
}

/*  ---------------------------  *
 *  -- PWM MEASUREMENT CLASS --  *
 *  ---------------------------  */
InputPWM::InputPWM(uint8_t pin, PwmRange range) : STM32HALPWM(pin) {
  if (range == LOW) 
    prescalerValue = 30;
  else 
    prescalerValue = 1;

  lastRising  = 0;
  lastFalling = 0;
  period      = 0;
  pulseWidth  = 0;
}

void InputPWM::handleCapture() {
  uint32_t capture = halTimer->getCaptureCompare(channel); 
  bool level = digitalRead(pwmPin); // true = rising, false = falling;

  uint32_t timerMax = halTimer->getHandle()->Instance->ARR;
  if (level) {
    if (lastRising != 0) {
      if (capture >= lastRising)
        period = capture - lastRising; 
      else 
        period = capture + (timerMax - lastRising + 1); 
    }
    lastRising = capture;
  }
  else {
    if (lastRising != 0) {
      if (capture >= lastRising) 
        pulseWidth = capture - lastRising; 
      else
        pulseWidth = capture + (timerMax - lastRising + 1); 
    }
    lastFalling = capture;
  }
}

void InputPWM::begin() {
  if (!halTimer) return;

  // register instance in lookup array
  Serial.print("outputId: "); 
  Serial.println(outputId);
  if (outputId < 16) inputInstances[outputId] = this;

  Serial.print("assignment test : ");
  Serial.print(inputInstances[outputId]->outputId);

  halTimer->setMode(channel, TIMER_INPUT_CAPTURE_BOTHEDGE, pwmPin);
  halTimer->setPrescaleFactor(prescalerValue); 

  if (outputId < 16)
    halTimer->attachInterrupt(channel, captureWrappers[outputId]);
  
  halTimer->resume();
  initialized = true;
}

void InputPWM::end() {
  if (halTimer) halTimer->detachInterrupt(channel);
  initialized = false;
}

float InputPWM::getFrequency() {
  if (period == 0 || !halTimer) return 0.0f;

  float timerClk = static_cast<float>(halTimer->getTimerClkFreq()) / static_cast<float>((prescalerValue)); 
  return timerClk / static_cast<float>(period);
}

float InputPWM::getDutyCycle() {
  if (period == 0) return 0.0f;

  return 100.0f * static_cast<float>(pulseWidth) / static_cast<float>(period);
}
