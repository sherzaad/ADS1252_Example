//example code for ADS1252 24-bit ADC with UNO R4 (minima/Wifi)

#include "pwm.h"

//constants derived from datasheet (https://www.ti.com/lit/ds/symlink/ads1252.pdf)
#define f_ADCLK 1000000UL  //1MHz
#define f_MCLK (f_ADCLK / 6)
#define f_DRATE (f_MCLK / 64)
#define p_MCLK 6              // 1/f_MCLK = 6us <---- need to update if f_ADSCLK changed
#define t_DRDY (14 * p_MCLK)  //(t2+t3) * p_MCLK, a little more that the minimum time, ;)
#define t_DOUT (348 * p_MCLK)
#define t_CONVCYCLE (384 * p_MCLK)
#define t_RESET ((5 * t_CONVCYCLE) / 1000)  //time in ms, a little more that the minimum time, 4 * t_CONVCYCLE ;)
#define ADC_BITS 24

//pin definitions
#define CLKPIN D6
#define ADC1_SCLKPIN D7
#define ADC1_DOUTPIN D2

// Instantiate a PwmOut object for pin D6
PwmOut pwm(CLKPIN);

//global variables
volatile int32_t ADC1_readin = 0;
volatile uint8_t intr_enabled = 1;

//function to read the ADC
void read_adc() {
  int32_t ADC_readin = 0;
  uint8_t t = p_MCLK / 2;

  // wait for DRDY to pass and to reach start-point of DOUT
  delayMicroseconds(t_DRDY);

  //bit banging to read in the adc 24 bits
  for (uint8_t i = 0; i < ADC_BITS; ++i) {
    delayMicroseconds(t);
    digitalWrite(ADC1_SCLKPIN, HIGH);
    delayMicroseconds(t);
    ADC_readin <<= 1;
    ADC_readin |= digitalRead(ADC1_DOUTPIN);
    digitalWrite(ADC1_SCLKPIN, LOW);
  }

  //update global variable
  ADC_readin <<= 8;          //shift <<8 as datatype is int32_t to get signed value of what was read in
  ADC_readin /= 256;         //divide by 256 (2^8) to get back the magnitude of what was read in
  ADC1_readin = ADC_readin;  

  intr_enabled = 0;
  detachInterrupt(digitalPinToInterrupt(ADC1_DOUTPIN));
}

void setup() {
  Serial.begin(115200);

  // Configure the PWM pin for 1,000,000 Hz frequency (1 us period)
  // and an initial duty cycle of 50.0%
  // The 'begin' method takes frequency (float) and initial duty cycle (float percentage)
  float freq = f_ADCLK * 1.0f;
  pwm.begin(freq, 50.0f);

  //configure pins
  pinMode(ADC1_SCLKPIN, OUTPUT);
  pinMode(ADC1_DOUTPIN, INPUT_PULLUP);

  // to reset ADC, we need SCLK HIGH for min of 4 CONVCYCLES
  digitalWrite(ADC1_SCLKPIN, HIGH);
  delay(t_RESET);
  digitalWrite(ADC1_SCLKPIN, LOW);

  //enable  interrupt to start ADC read
  attachInterrupt(digitalPinToInterrupt(ADC1_DOUTPIN), read_adc, FALLING);
}

void loop() {
  if (intr_enabled == 0) {
    Serial.println(ADC1_readin);
    intr_enabled = 1;
    //enable  interrupt for ADC read
    attachInterrupt(digitalPinToInterrupt(ADC1_DOUTPIN), read_adc, FALLING);
  }
}
