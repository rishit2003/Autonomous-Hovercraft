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


ISR(ADC_vect) {

  // ADC conversion complete
  // Clear ADIF by writing one to it
    ADCSRA |= (1<<ADIF);

 //uint8_t adc_valueh = ADCH;
//  uint8_t adc_valuel = ADCL;
my_adc=ADCH;
  
  //counter++;  //how many measurement was done
   
}


void adc_init_rev() {
  // ADC init

  //---ADMUX=((0<<REFS1)|(0<<REFS0)); // set voltage refere as Aref

  ADMUX = ((1 << ADLAR) | (1<<MUX1)|(1<<MUX0) | (0 << REFS1) | (1 << REFS0));  // "left-aligned" result for easy 8-bit reading.
                                                                            // AVcc as Aref |(1<<REFS0)
                                                                            // Sets ADC to the specified channel. Can be changed later.
  ADCSRB = 0;                                                               // ADTS[0-2]=0 - Free Running mode   		//---- new code added
  ADCSRA = (1 << ADEN)|(1 << ADIE); //-----   |=                                                     //enable ADC

 // ADCSRA |= (1 << ADIE);  // enable ADC Complete Interrupt. NOTE: the ISR MUST be defined!!!

  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  // ADC clock prescaler 128 or sampling frequency
  ADCSRA |= (1 << ADATE);                                // Continuosly running mode //auto-triggering The ADC will start a conversion on a positive edge of
                                                         // the selected trigger signal. The trigger source is selected by setting the ADC trigger select bits, ADTS in ADCSRB.
  ADCSRA |= (1 << ADSC);                                 // Start ADC
}


uint16_t read_adc() {

  uint8_t Low, High;
  //ADMUX = (ADMUX & (ADC_VOLTAGE_REFERENCE_MASK)) | Pin;

  //ADMUX = (ADMUX | ((pin - 14) & 0x07));  //----  |= instead
                                          // Delay needed for the stabilization of the ADC input voltage
  //_delay_us(10);
  ADCSRA |= (1 << ADSC);  //start conversion
 // while (ADCSRA & (1 << ADSC))

 // Wait for ADC conversion to complete
 while((ADCSRA & (1<<ADIF)) == 0);

  //wait conversion is complete
  Low = ADCL;
  High = ADCH;
  	//return (High<<8) | Low;  // ------return ADCH
  return HIGH;  // ------return ADCH
}

//  This bit is set when an 
// ADC conversion completes and the Data Registers are updated

int main(void) {

  Serial.begin(9600);


  //analogReference(DEFAULT);  //-------- to be modified

  DDRB &= ~(1 << PC3);   //changing the direction of pin A3 to input and the rest unchanged
  PORTB &= ~(1 << PC3);  // disable pull-up resistor
  //sei();


  adc_init_rev();
  sei();

  

  while (1) {

     read_adc();
    Vin = (float)my_adc*5/256;
    my_distance= (Vin-0.0857)/21.472;

    Serial.println(my_adc);
     Serial.print("voltage is: ");
    Serial.println(Vin);
Serial.print("distance is: ");
    Serial.println(my_distance,3);    //inverse distance
    


    _delay_ms(500);
  }


  return 0;
}
