#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm8s003.h"
#include "stm8_utility.h"
#include "uart.h"

int putchar(int c) {
    uart_write(c);
    return 0;
}

//TONI
#define TONE_C6 956
#define TONE_D6 851
#define TONE_E6 758
#define TONE_C7 478

//pwm pin -> PD2
#define PWM_PORT_CR1 PD_CR1
#define PWM_PORT_DDR PD_DDR
#define PWM_PORT_ODR PD_ODR
#define PWM_PIN 2

//dir pin -> PC6
#define DIR_PORT_CR1 PC_CR1
#define DIR_PORT_DDR PC_DDR
#define DIR_PORT_ODR PC_ODR
#define DIR_PIN 6

//encoder inputs -> PC4, PC5
#define ENCODER_PORT_CR1 PC_CR1
#define ENCODER_PORT_CR2 PC_CR2
#define ENCODER_PORT_DDR PC_DDR
#define ENCODER_PORT_ODR PC_ODR
#define ENCODER_PORT_IDR PC_IDR
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5

#define CAPTURE_PORT_CR1 PD_CR1
#define CAPTURE_PORT_CR2 PD_CR2
#define CAPTURE_PORT_DDR PD_DDR
#define CAPTURE_PORT_ODR PD_ODR
#define CAPTURE_PORT_IDR PD_IDR
#define CAPTURE_PIN 4

const volatile int8_t rotacijska_tabela[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
volatile int16_t enc_cnt = 0;
volatile uint8_t last_ab = 0;

volatile int toner = 0;

void enc_interrupt() __interrupt(EXTI2_ISR) {
  //PB_IDR
  uint8_t ab = !(ENCODER_PORT_IDR & _BV(ENCODER_PIN_A)) << 1 | !(ENCODER_PORT_IDR & _BV(ENCODER_PIN_B));
  int8_t iz_tabele = rotacijska_tabela[last_ab << 2 | ab];
  enc_cnt += iz_tabele;
  last_ab = ab;
}

void encoder_input_init() {
  ENCODER_PORT_DDR &= ~(_BV(ENCODER_PIN_B) | _BV(ENCODER_PIN_A));
  ENCODER_PORT_CR2 |= (_BV(ENCODER_PIN_B) | _BV(ENCODER_PIN_A));
}

void tone(uint16_t t0, int16_t ms_duration) {
  //prvo shranimo nastavitve timerja2
  uint16_t pscr = TIM2_PSCR;
  uint16_t top = TIM2_ARRH << 8 | TIM2_ARRL;
  uint16_t duty = TIM2_CCR3H << 8 | TIM2_CCR3L;
  //shranimo nastavitev dir pina
  uint8_t dir_pin = DIR_PORT_ODR & _BV(DIR_PIN);

  //nastavimo nastavitve za zvok
  DIR_PORT_ODR &= ~_BV(DIR_PIN);
  TIM2_PSCR = 4; //16x division

  TIM2_ARRH = t0>>8;
  TIM2_ARRL = t0;

  uint16_t new_duty = t0>>3;
  TIM2_CCR3H = new_duty >> 8;
  TIM2_CCR3L = new_duty;

  //wait while the tone plays
  while(ms_duration-- > 1)
    util_delay_milliseconds(1);

  //restore settings
  TIM2_PSCR = pscr;

  TIM2_ARRH = top >> 8;
  TIM2_ARRL = top;

  TIM2_CCR3H = duty >> 8;
  TIM2_CCR3L = duty;

  DIR_PORT_ODR |= dir_pin;
}

/*
void timer1_pwm_init() {
  TIM1_BKR |= _BV(7); //MOE , timer output enable

  //Timer
  TIM1_PSCRH = 0;
  TIM1_PSCRL = 8;

  TIM1_ARRH = 0;
  TIM1_ARRL = 255; //Max / Top

  //CH2

  //TIM2_IER |= _BV(TIM2_IER_UIE);
  TIM1_CR1 |= _BV(TIM2_CR1_CEN);

  TIM1_CCMR1 |=  (0b110 << 4); //pwm mode 1 //bita 0 in 1 moreta bit na nic ce ces met izhod
  TIM1_CCER1 |= _BV(0); //< treba je nastavit za 1. kanal

  TIM1_CCR1H = 0;
  TIM1_CCR1L = 200; //Duty
}
*/
volatile uint16_t delta = 0;
volatile uint16_t tmp = 0;
void input_capture() __interrupt(TIM2_CC_ISR) {
  uint8_t end = TIM2_SR1 & _BV(1);//CC1IF
  uint16_t ccr = TIM2_CCR1H << 8 | TIM2_CCR1L;

  //PD_ODR ^= _BV(3);

  if(CAPTURE_PORT_IDR & _BV(CAPTURE_PIN)) {
    TIM2_CCER1 |= _BV(1);
    tmp = ccr;
    //PD_ODR |= _BV(3);
  }
  else {
    if(ccr > tmp)
      delta = ccr-tmp;
    else
      delta = ccr-tmp+4095;
    TIM2_CCER1 &= ~_BV(1);
    //PD_ODR &= ~_BV(3);
  }
}

void timer2_pwm_init() {
  //Timer
  TIM2_PSCR = 0b100;

  TIM2_ARRH = 4095 >> 8;
  TIM2_ARRL = 4095 & 0xff; //Max / Top

  TIM2_CCMR3 |=  (0b110 << 4); //pwm mode 1 //bita 0 in 1 moreta bit na nic ce ces met izhod
  TIM2_CCER2 |= _BV(0); //< treba je nastavit za 3. kanal

  TIM2_CCR3H = 0;
  TIM2_CCR3L = 0; //Duty
}
void timer2_capture_init() {
  TIM2_IER |= _BV(1); //CC1IE
  TIM2_EGR |= _BV(1);//CC1G
  TIM2_CCMR1 |= _BV(0) | (0b1111<<4); //4x filter, TI1FP1
  TIM2_CCER1 |= _BV(0); //Enable capture
  //TIM1_PSCR |= 0xffff; //<- IDK nevem zakaj ne dela

  TIM2_CR1 |= _BV(TIM2_CR1_CEN);
}

void outputs_init() {
  DIR_PORT_DDR |= _BV(DIR_PIN);
  DIR_PORT_CR1 |= _BV(DIR_PIN);

  PWM_PORT_DDR |= _BV(PWM_PIN);
  PWM_PORT_CR1 |= _BV(PWM_PIN);
}

void set_output(uint8_t power, int8_t dir) {
  if(power == 0) {
    TIM2_CCR3H = 0;
    TIM2_CCR3L = 0;
    DIR_PORT_ODR &= ~_BV(DIR_PIN);
  }
  else if(dir <= 0) {
    DIR_PORT_ODR &= ~_BV(DIR_PIN);
    TIM2_CCR3H = power >> 4;
    TIM2_CCR3L = power << 4;
  }else {
    DIR_PORT_ODR |= _BV(DIR_PIN);
    power = 255 - power;
    TIM2_CCR3H = power >> 4;
    TIM2_CCR3L = power << 4;
  }
}

volatile uint16_t capture_value  = 0;
/*
void input_capture() __interrupt(TIM2_CC_ISR) {
  static uint16_t tmp = 0;
  uint8_t end = TIM2_SR1 & _BV(1);//CC1IF
  uint16_t ccr = TIM2_CCR1H << 8 | TIM2_CCR1L;

  if(PD_IDR & _BV(4)) {
    TIM2_CCER1 |= _BV(1);
    tmp = ccr;
  }
  else {
    capture_value = ccr - tmp;
    if(capture_value > 2100)
      capture_value = 1000;

    TIM2_CCER1 &= ~_BV(1);
  }
}
void timer2_input_capture_init() {
  TIM2_IER |= _BV(1); //CC1IE
  TIM2_EGR |= _BV(1);//CC1G
  TIM2_CCMR1 |= _BV(0) | (0b1111<<4); //4x filter, TI1FP1
  TIM2_CCER1 |= _BV(0); //Enable capture
  TIM2_PSCR |= 0b0100;

  TIM2_CR1 |= _BV(TIM2_CR1_CEN) | _BV(TIM2_CR1_ARPE);
}
*/

int main () {
    CLK_CKDIVR = 0;//16mhz

    CPU_CCR |= _BV(5) | _BV(3);
    EXTI_CR1 = 0xff; //rising and falling edge for interrupts

    encoder_input_init();
    //timer1_pwm_init();
    timer2_pwm_init();
    timer2_capture_init();
    outputs_init();

    CAPTURE_PORT_CR1 |= _BV(CAPTURE_PIN);
    uart_init(9600);

    enable_interrupts();

    tone(TONE_C7, 500);

    int16_t target = 5400; //max 30000 -> 5 krogov v vsako smerss

    /*while(1) {
      tone(delta, 50);
    }
    //*/

    //target = 0;
    /*while(1) {
      if(toner) {
        toner--;
        tone(TONE_C6, 5);
      }
    }
    */


    while(1) {
      if(delta < 1000) delta = 1000;
      target = (delta-1000) * 5.4f;

      uint16_t error = abs(target - enc_cnt);

      if(error > 500) {
        set_output(255, enc_cnt < target);
      }
      else if(error > 25) {
        set_output(190, enc_cnt < target);
      }
      else {
        set_output(0, 0);
      }

      //printf("e %d d %u\n\r", enc_cnt, delta);
    }
    //*/

}