#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "init_290.c"

//volatile unsigned int time_start;
//volatile unsigned int length;
volatile int TimerOverflow = 0;
int pinB5_status;
//char my_buffer[10];

#define F_CPU 16000000UL
//#define PRESCALER 1



void D3_brightness_ultrasonic(float distance) {
  if (distance > 40) {
    OCR2A = 0;
  } else if ((distance <= 40) & (distance >= 15)) {
    OCR2A = (255 - (float)distance * 6);   //PWM on the OC2A pin 
  } else {
    OCR2A = 255;
  }
}





ISR(TIMER1_OVF_vect) {
  TimerOverflow++; /* Increment Timer Overflow count */
}

// ISR(TIMER1_CAPT_vect) {


// }

int main(void) {
  Serial.begin(9600);

  DDRB |= (1 << PB5);    // Trig   onboard  yellow led
  DDRB &= ~(1 << PB0);   // Echo
  PORTB &= ~(1 << PB0);  //making sure that internal pull-up resistor is not active.
                         //char string[10];
  DDRB |= (1 << PB3);    //      red led

  long count;
  float distance;
  sei(); /* Enable global interrupt */

  TIMSK1 = (1 << TOIE1); /* Enable Timer1 overflow interrupts */
  TCCR1A = 0;            /* Set all bit to zero Normal operation */

  //timer2_init();

  //DDRB |= (1 << PB3);   // modified DDRD |= (1 << PB3);

  OCR2A = 100;
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << WGM21) | (1 << WGM20);   // fast pwm, non inverting mode
  TCCR2B = (1 << CS22) | (1 << CS20);   //128 prescaler

  while (1) {
    /* Give 10us trigger pulse on trig. pin to HC-SR04 */
    PORTB &= ~(1 << PB5);
    _delay_us(1);
    PORTB |= (1 << PB5);
    _delay_us(10);
    PORTB &= ~(1 << PB5);

    _delay_ms(0.25);        // LED to be triggered if needed only after the 8 ,40Khz, bursts (0.20ms) for the ultrasonic sensor
    PORTB |= pinB5_status;  // get previous pin status before the triggering of the 10us pulse for the ultasonic sensor


    TCNT1 = 0;         /* Clear Timer counter */
    TCCR1B = 0x41;     /* Capture on rising edge, No prescaler*/
    TIFR1 = 1 << ICF1; /* Clear ICP flag (Input Capture flag) */
    TIFR1 = 1 << TOV1; /* Clear Timer Overflow flag */

    /*Calculate width of Echo by Input Capture (ICP) */

    while ((TIFR1 & (1 << ICF1)) == 0)
      ;                /* Wait for rising edge */
    TCNT1 = 0;         /* Clear Timer counter */
    TCCR1B = 0x01;     /* Capture on falling edge, No prescaler */
    TIFR1 = 1 << ICF1; /* Clear ICP flag (Input Capture flag) */
    TIFR1 = 1 << TOV1; /* Clear Timer Overflow flag */
    TimerOverflow = 0; /* Clear Timer overflow count */

    while ((TIFR1 & (1 << ICF1)) == 0)
      ;                                     /* Wait for falling edge */
    count = ICR1 + (65535 * TimerOverflow); /* Take count */
    //  if (count >= 37318 or count <= 13994) {
    //     PORTB |= 1 << 5;  // Writing HIGH to PB5
    //     //_delay_ms(1000);  // Delay of 1 Second
    //   } else {
    //     PORTB &= ~(1 << 5);  // Writing LOW to PB5
    //    //_delay_ms(1000);     // Delay of 1 Second
    //   }

    distance = (float)count * ((float)PRESCALER / F_CPU) * 17150;  // speed in cm per sec
    Serial.print(count);
    Serial.println();

    Serial.print(distance);
    Serial.println(" cm");

    //uart_transmit_char("/n");
    // uart_transmit_string("the  distance to the object is:");
    // sprintf(my_buffer,"%.2f",distance);
    // uart_transmit_string(my_buffer);
    // uart_transmit_string(" cm");

    _delay_ms(10);

    D3_brightness_ultrasonic(distance);


    // turnin the onboard yellow LED ON or OFF depending on specific range
    if ((distance >= 40) | (distance <= 15)) {

      PORTB |= (1 << 5);  // Writing HIGH to PB5

      // Delay of 1 Second
    } else {
      PORTB &= ~(1 << 5);  // Writing LOW to PB5

      // _delay_ms(100);     // Delay of 1 Second
    }


    _delay_ms(100);
    pinB5_status = (PINB & (1 << PB5));  // get pin B5 status to be sure it stays ON or OFF right after triggering the ultrasonic sensor
  }
}
