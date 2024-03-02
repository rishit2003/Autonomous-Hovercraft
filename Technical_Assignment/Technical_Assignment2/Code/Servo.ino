#include "init_290.h"
#include <util/delay.h>

#define PWM_TOP 500  // Adjust this value based on your requirements

const uint16_t Servo_angle [256]={ //  +/-90 degrees
85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, // 0...15
108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108, // 16...31 0.9ms pulse(108) 
119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119, // 32...47
131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131, // 48...63
142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142, // 64...79
154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154, // 80...95
165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165, // 96...111
177,177,177,177,177,177,177,177,177,177,177,177,188,188,188,188, // center - 1.5ms pulse (188) 112...127
188,188,188,188,198,198,198,198,198,198,198,198,198,198,198,198, // 128...143
208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208, // 143...159
218,218,218,218,218,218,218,218,218,218,218,218,218,218,218,218, // 160...175
228,228,228,228,228,228,228,228,228,228,228,228,228,228,228,228, // 175...191
238,238,238,238,238,238,238,238,238,238,238,238,238,238,238,238, // 192...207
248,248,248,248,248,248,248,248,248,248,248,248,248,248,248,248, // 208...223
270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270, // 224...239
290,290,290,290,290,290,290,290,290,290,290,290,290,290,290,290  // 240...255 
};

void timer1_50Hz_init(uint8_t en_IRQ) {
   // Set OC1A (pin 9 on ATmega328P) as output
    DDRB |= (1 << DDB1);
    TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Non-inverted PWM on channels A and B
    TCCR1B |= (1 << WGM13);  // PWM, Phase and Frequency Correct. TOP=ICR1.
    ICR1 = PWM_TOP; // 50Hz PWM
    OCR1A = Servo_angle[127]; 
    OCR1B = 0; 
    TCCR1B |= (1 << CS11) | (1 << CS10); // Timer prescaler
    // Set initial duty cycle to 0
    OCR1A = 0;
    if (en_IRQ) TIMSK1 |= (1 << ICIE1); // Enable Input Capture Interrupt. Note: The ISR MUST be defined!!! 
}

ISR(TIMER1_CAPT_vect) {
    // Your ISR code here
}

void set_servo_position(uint8_t position) {
    uint16_t pulse_width = Servo_angle[position];
    OCR1A = pulse_width;
    _delay_ms(30);
}

int main() {
    // Initialize PWM and enable Input Capture Interrupt
    timer1_50Hz_init(1);
    Serial.begin(9600);

    // Enable global interrupts
    sei();

    // Main loop
    while (1) {
        // Move the servo from 0 to 180 degrees
        for (uint8_t pos = 0; pos <= 180; pos++) {
            set_servo_position(pos);
            Serial.println(pos);
        }

        // Move the servo from 180 to 0 degrees
        for (uint8_t pos = 180; pos > 0; pos--) {
            set_servo_position(pos);
            Serial.println(pos);
        }
    }

    return 0;
}
