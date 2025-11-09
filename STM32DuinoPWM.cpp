#include "STM32DuinoPWM.hpp"

static InputPWM *inputInstances[16] = {nullptr};

static void captureWrapper0()  { if (inputInstances[0])   inputInstances[0]->handleCapture(); }
static void captureWrapper1()  { if (inputInstances[1])   inputInstances[1]->handleCapture(); }
static void captureWrapper2()  { if (inputInstances[2])   inputInstances[2]->handleCapture(); }
static void captureWrapper3()  { if (inputInstances[3])   inputInstances[3]->handleCapture(); }
static void captureWrapper4()  { if (inputInstances[4])   inputInstances[4]->handleCapture(); }
static void captureWrapper5()  { if (inputInstances[5])   inputInstances[5]->handleCapture(); }
static void captureWrapper6()  { if (inputInstances[6])   inputInstances[6]->handleCapture(); }
static void captureWrapper7()  { if (inputInstances[7])   inputInstances[7]->handleCapture(); }
static void captureWrapper8()  { if (inputInstances[8])   inputInstances[8]->handleCapture(); }
static void captureWrapper9()  { if (inputInstances[9])   inputInstances[9]->handleCapture(); }
static void captureWrapper10() { if (inputInstances[10]) inputInstances[10]->handleCapture(); }
static void captureWrapper11() { if (inputInstances[11]) inputInstances[11]->handleCapture(); }
static void captureWrapper12() { if (inputInstances[12]) inputInstances[12]->handleCapture(); }
static void captureWrapper13() { if (inputInstances[13]) inputInstances[13]->handleCapture(); }
static void captureWrapper14() { if (inputInstances[14]) inputInstances[14]->handleCapture(); }
static void captureWrapper15() { if (inputInstances[15]) inputInstances[15]->handleCapture(); }

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

int getChannelForPin(uint8_t pin) {
    for (auto &m : timerMap) {
        if (m.pin == pin) return m.channel; 
    }
    return -1;
}

/*  ---------------------------  *
 *  -- BASE TIMER CLASS      --  *
 *  ---------------------------  */
STM32HALPWM::STM32HALPWM(uint8_t pin) 
  : halTimer(nullptr), pwmPin(pin), channel(0), initialized(false) {

  TIM_TypeDef *timer = getTimerForPin(pin);
  channel = getChannelForPin(pin);
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

uint8_t STM32HALPWM::getChannel() const {
  return channel;
}


/*  ---------------------------  *
 *  -- PWM GENERATOR CLASS   --  *
 *  ---------------------------  */
OutputPWM::OutputPWM(uint8_t pin, uint32_t freq, float dutyCycle)
  : STM32HALPWM(pin), frequency(freq), dutyCycle(dutyCycle) {}

void OutputPWM::begin() {
  Serial.println("output begin called");
  halTimer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pwmPin);
  Serial.println("setmode success");
  halTimer->setOverflow(frequency, HERTZ_FORMAT);
  Serial.println("set freq success");
  halTimer->setCaptureCompare(channel, 0, PERCENT_COMPARE_FORMAT);
  Serial.println("set dc success");
  halTimer->resume(); 
  Serial.println("resume success");
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
  if (channel < 16) inputInstances[channel] = this;

  halTimer->setMode(channel, TIMER_INPUT_CAPTURE_BOTHEDGE, pwmPin);
  halTimer->setPrescaleFactor(prescalerValue); 

  switch (channel) {
    case  0: halTimer->attachInterrupt( 0,  captureWrapper0); break; 
    case  1: halTimer->attachInterrupt( 1,  captureWrapper1); break; 
    case  2: halTimer->attachInterrupt( 2,  captureWrapper2); break; 
    case  3: halTimer->attachInterrupt( 3,  captureWrapper3); break; 
    case  4: halTimer->attachInterrupt( 4,  captureWrapper4); break; 
    case  5: halTimer->attachInterrupt( 5,  captureWrapper5); break; 
    case  6: halTimer->attachInterrupt( 6,  captureWrapper6); break; 
    case  7: halTimer->attachInterrupt( 7,  captureWrapper7); break; 
    case  8: halTimer->attachInterrupt( 8,  captureWrapper8); break; 
    case  9: halTimer->attachInterrupt( 9,  captureWrapper9); break; 
    case 10: halTimer->attachInterrupt(10, captureWrapper10); break; 
    case 11: halTimer->attachInterrupt(11, captureWrapper11); break; 
    case 12: halTimer->attachInterrupt(12, captureWrapper12); break; 
    case 13: halTimer->attachInterrupt(13, captureWrapper13); break; 
    case 14: halTimer->attachInterrupt(14, captureWrapper14); break; 
    case 15: halTimer->attachInterrupt(15, captureWrapper15); break; 
  }
  
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
