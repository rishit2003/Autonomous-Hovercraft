#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "init_290.c"


#define F_CPU 16000000UL

uint16_t my_adc;
float my_distance;

float Vin;


void D3_brightness_infrared(uint16_t adc_value) {
  if (adc_value > 80) {  // ADC value of 35 represent voltage around 40cm , ADC value of 80 represent voltage around 15cm-
                         // they were measured at first with ir sensor
    OCR2A = 255;
  } else if ((adc_value <= 80) & (adc_value >= 35)) {  // PWM on the OC2A pin
    OCR2A = ( 255- (float)adc_value *2);  // scaling factor in front of adc_value to amplify the brightness during PWM
  } else {
    OCR2A = 0;
  }
}

void onboard_LED_brightness_infrared(uint16_t adc_value) {

  if ((adc_value >= 80) | (adc_value <= 35)) {

    PORTB |= (1 << 5);  // Writing HIGH to PB5

    // Delay of 1 Second
  } else {
    PORTB &= ~(1 << 5);  // Writing LOW to PB5

    // _delay_ms(100);     // Delay of 1 Second
  }
}

ISR(ADC_vect) {

  // ADC conversion complete
  // Clear ADIF by writing one to it
  ADCSRA |= (1 << ADIF);

  //uint8_t adc_valueh = ADCH;
  //  uint8_t adc_valuel = ADCL;
  my_adc = ADCH;

  //counter++;  //how many measurement was done
}


void adc_init_rev() {
  // ADC init

  //---ADMUX=((0<<REFS1)|(1<<REFS0)); // set voltage refere as Aref

  ADMUX = ((1 << ADLAR) | (1 << MUX1) | (1 << MUX0) | (0 << REFS1) | (1 << REFS0));  // "left-aligned" result for easy 8-bit reading.
                                                                                     // AVcc as Aref |(1<<REFS0)
                                                                                     // Sets ADC to the specified channel. Can be changed later.
  ADCSRB = 0;                                                                        // ADTS[0-2]=0 - Free Running mode   		//---- new code added
  ADCSRA = (1 << ADEN) | (1 << ADIE);                                                //-----   |=                                                     //enable ADC

  // ADCSRA |= (1 << ADIE);  // enable ADC Complete Interrupt. NOTE: the ISR MUST be defined!!!

  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // ADC clock prescaler 128 or sampling frequency
  ADCSRA |= (1 << ADATE);                                // Continuosly running mode //auto-triggering The ADC will start a conversion on a positive edge of
                                                         // the selected trigger signal. The trigger source is selected by setting the ADC trigger select bits, ADTS in ADCSRB.
  ADCSRA |= (1 << ADSC);                                 // Start ADC
}


uint16_t read_adc() {

  uint8_t Low, High;
  
  ADCSRA |= (1 << ADSC);  //start conversion
  // while (ADCSRA & (1 << ADSC))

  // Wait for ADC conversion to complete
  while ((ADCSRA & (1 << ADIF)) == 0)
    ;

  //wait conversion is complete
  Low = ADCL;
  High = ADCH;
  
  return HIGH;  // ------return ADCH
}


int main(void) {

  Serial.begin(9600);

  DDRB |= (1 << PB5);    //  set port as output for  onboard  yellow led
  DDRC &= ~(1 << PC3);   //changing the direction of pin A3 to input and the rest unchanged
  PORTC &= ~(1 << PC3);  // disable pull-up resistor

  DDRB |= (1 << PB3);  // set port as output for red led 



  adc_init_rev();
  sei();


  OCR2A = 100;
  TCCR2A = (1 << COM2A1) | (0 << COM2A0) | (1 << WGM21) | (1 << WGM20);   // fast pwm, non inverting mode
  TCCR2B = (1 << CS22) | (1 << CS20);   //128 prescaler


  while (1) {

    read_adc();
    Vin = (float)my_adc * 5 / 256;  // Vref should be 2.6 not 5 and we use 256 because we take the highest 8 bits for the ADC values (left aligned)
     
    my_distance = (Vin - 0.0857) / 21.472;  //inverse distance estimated from plot on excel we did 

    Serial.println(my_adc);
    Serial.print("voltage is: ");
    Serial.println(Vin);
    Serial.print("inverse distance is: ");
    Serial.println(1/my_distance, 3);  //inverse distance


   D3_brightness_infrared(my_adc);
    onboard_LED_brightness_infrared(my_adc);



    _delay_ms(500) ;
    }


  return 0;
}
