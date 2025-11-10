#ifndef _STM32DUINOPWMPINDEFH
#define _STM32DUINOPWMPINDEFH
#include "HardwareTimer.h"
#include <Arduino.h>

#ifdef F401RE
#include "stm32f401xx.h"
#endif 

#ifdef G0B1RE
#include "stm32g0b1xx.h"
#endif 

#ifdef G474RE
#include "stm32g474xx.h"
#endif

#ifdef H753ZI
#include "stm32h753xx.h"
#endif

#ifdef L412KB
#include "stm32l412xx.h"
#endif 

#ifdef L432KC
#include "stm32l432xx.h"
#endif


struct PinTimerMap {
    uint8_t pin;
    TIM_TypeDef *timer;
    uint8_t channel;
};

static const PinTimerMap timerPinChannels[] = {
  #ifdef F401RE
    {PA8,  TIM1, 1}, //D7 
    {PA9,  TIM1, 2}, //D8
    {PA10, TIM1, 3}, //D2
    {PA11, TIM1, 4},
    {PA0,  TIM2, 1}, //A0
    {PA1,  TIM2, 2}, //A1
    {PB10, TIM2, 3}, //D6
    {PA6,  TIM3, 1}, //D12
    {PA7,  TIM3, 2}, //D11
    {PB0,  TIM3, 3}, //A3
    {PB1,  TIM3, 4}, 
    {PB6,  TIM4, 1}, //D10
    {PB7,  TIM4, 2},
    {PB8,  TIM4, 3}, //D15
    {PB9,  TIM4, 4}, //D14
  #endif
  #ifdef G0B1RE
    {PA8,  TIM1, 1}, //
    {PA9,  TIM1, 2}, //
    {PA10, TIM1, 3}, //
    {PC11, TIM1, 4}, // 
    {PA0,  TIM2, 1}, //
    {PA1,  TIM2, 2}, //
    {PB10, TIM2, 3}, //
    {PB11, TIM2, 4}, //
    {PA6,  TIM3, 1}, //
    {PA7,  TIM3, 2}, //
    {PB0,  TIM3, 3}, //
    {PB1,  TIM3, 4}, //
    {PB6,  TIM4, 1}, //
    {PB7,  TIM4, 2}, //
    {PB8,  TIM4, 3}, //
    {PB9,  TIM4, 4}, //
  #endif
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
  #ifdef H753ZI
    {PE9,  TIM1, 1}, //
    {PE11, TIM1, 2}, //
    {PE13, TIM1, 3}, //
    {PE14, TIM1, 4}, //
    {PA5,  TIM2, 1}, //
    {PB3,  TIM2, 2}, //
    {PB10, TIM2, 3}, //
    {PB11, TIM2, 4}, //
    {PA6,  TIM3, 1}, //
    {PA7,  TIM3, 2}, //
    {PC8,  TIM3, 3}, //
    {PB1,  TIM3, 4}, //
    {PD12, TIM4, 1}, //
    {PD13, TIM4, 2}, //
    {PD14, TIM4, 3}, //
    {PD15, TIM4, 4}, //
    {PA0,  TIM5, 1}, //
    {PA1,  TIM5, 2}, //
    {PA2,  TIM5, 3}, //
    {PA3,  TIM5, 4}, //

  #endif
  #if defined(L412KB) || defined(L432KC)
    {PA8,  TIM1, 1}, //
    {PA9,  TIM1, 2}, //
    {PA10, TIM1, 3}, //
    {PA11, TIM1, 4}, //
    {PA5,  TIM2, 1}, //
    {PA1,  TIM2, 2}, //
    {PA3,  TIM2, 4}, //
  #endif 
};
#endif // !_STM32DUINOPWMPINDEFH
