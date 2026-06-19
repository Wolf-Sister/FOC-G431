/**
  ******************************************************************************
  * @file    Arduino.h
  * @brief   Minimal Arduino compatibility header for SimpleFOC on bare-metal STM32
  ******************************************************************************
  */
#ifndef ARDUINO_H
#define ARDUINO_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- STM32 HAL + LL ---- */
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"

/* ---- Standard C headers ---- */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- Arduino type aliases ---- */
typedef uint8_t  byte;
typedef uint16_t word;
typedef bool     boolean;

/* ---- PinMap (Arduino STM32 convention) ---- */
typedef uint32_t PinName;
#define NC  0xFFFFFFFFUL

typedef struct {
    PinName pin;
    uint32_t peripheral;
    uint32_t function;
} PinMap;

#define STM_PIN_FUNCTION(af, ch, inv)  (((af) << 4) | ((ch) & 0x0F) | ((inv) ? 0x8000 : 0))
#define STM_PIN_CHANNEL(fn)    ((fn) & 0x0F)
#define STM_PIN_ALT(fn)        (((fn) >> 4) & 0x0F)
#define STM_PIN_INVERTED(fn)   (((fn) >> 15) & 0x01)
#define ALTX_MASK  0

/* ---- Pin number encoding ---- */
#define PIN(port, num)  (((port) << 4) | (num))
#define PORT_A  0
#define PORT_B  1

#define PA0   PIN(PORT_A, 0)
#define PA1   PIN(PORT_A, 1)
#define PA4   PIN(PORT_A, 4)
#define PA5   PIN(PORT_A, 5)
#define PA6   PIN(PORT_A, 6)
#define PA7   PIN(PORT_A, 7)
#define PA8   PIN(PORT_A, 8)
#define PA9   PIN(PORT_A, 9)
#define PA10  PIN(PORT_A, 10)
#define PB9   PIN(PORT_B, 9)
#define PB10  PIN(PORT_B, 10)
#define PB11  PIN(PORT_B, 11)
#define PB12  PIN(PORT_B, 12)
#define PB13  PIN(PORT_B, 13)
#define PB14  PIN(PORT_B, 14)
#define PB15  PIN(PORT_B, 15)

/* ---- Arduino pin constants ---- */
#define HIGH  1
#define LOW   0
#define OUTPUT       1
#define INPUT        0
#define INPUT_PULLUP 2
#define CHANGE       1
#define FALLING      2
#define RISING       3

/* ---- SPI constants ---- */
#define SPI_MODE0  0
#define SPI_MODE1  1
#define SPI_MODE2  2
#define SPI_MODE3  3
#define MSBFIRST   1
#define LSBFIRST   0

/* ---- Math constants ---- */
#ifndef PI
#define PI      3.14159265358979323846f
#endif
#ifndef TWO_PI
#define TWO_PI  6.28318530717958647693f
#endif
#ifndef HALF_PI
#define HALF_PI 1.57079632679489661923f
#endif

/* ---- Utility macros ---- */
#ifndef abs
#define abs(x)  ((x) > 0 ? (x) : -(x))
#endif
#ifndef min
#define min(a,b)  ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a,b)  ((a) > (b) ? (a) : (b))
#endif
#define round(x)  ((int)((x) + 0.5f))

#define NOT_SET  (-12345.0f)

/* ---- Arduino core prototypes ---- */
PinName  digitalPinToPinName(int pin);
PinName  analogInputToPinName(int pin);
void*    pinmap_peripheral(PinName pin, const PinMap* map);
uint32_t pinmap_function(PinName pin, const PinMap* map);
void     pinmap_pinout(PinName pin, const PinMap* map);
void     pinMode(int pin, int mode);
void     digitalWrite(int pin, int value);
int      digitalRead(int pin);
int      analogRead(int pin);
void     analogWrite(int pin, int value);
void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);
uint32_t millis(void);
uint32_t micros(void);
void     enableTimerClock(TIM_HandleTypeDef *htim);

extern PinMap PinMap_TIM[];
extern PinMap PinMap_ADC[];

#ifdef __cplusplus
} // extern "C"

// getTimerClkSrc — defined in stm32_timerutils.cpp (C++ linkage)

uint8_t getTimerClkSrc(TIM_TypeDef *tim);

/* ---- C++ Arduino classes ---- */

class __FlashStringHelper;

class Print {
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t c) { (void)c; return 1; }
    virtual size_t write(const uint8_t *buf, size_t size) {
        for (size_t i = 0; i < size; i++) write(buf[i]);
        return size;
    }
    size_t print(const char *s)       { return write((const uint8_t*)s, strlen(s)); }
    size_t print(const __FlashStringHelper*) { return 0; }
    size_t println(const char *s)     { size_t n=print(s); write('\r'); write('\n'); return n+2; }
    size_t print(int v)               { char b[16]; snprintf(b,16,"%d",v);    return write((uint8_t*)b,strlen(b)); }
    size_t print(float v, int d=2)    { char b[32]; snprintf(b,32,"%.*f",d,v); return write((uint8_t*)b,strlen(b)); }
    size_t println(int v)             { size_t n=print(v); write('\r'); write('\n'); return n+2; }
    size_t println(float v)           { size_t n=print(v); write('\r'); write('\n'); return n+2; }
    size_t println(void)              { write('\r'); write('\n'); return 2; }
    void   printf(const char*, ...)   {}
};

class Stream : public Print {
public:
    virtual int  available() { return 0; }
    virtual int  read()      { return -1; }
    virtual int  peek()      { return -1; }
    virtual void flush()     {}
    float parseFloat()       { return 0.0f; }
    int   parseInt()         { return 0; }
};

class HardwareSerial : public Stream {
public:
    void begin(unsigned long baud) { (void)baud; }
    void begin(unsigned long, uint32_t) {}
    operator bool() { return true; }
    virtual size_t write(uint8_t c) { (void)c; return 1; }
};

#include "spi_bridge.h"

#endif /* __cplusplus */

#endif /* ARDUINO_H */
