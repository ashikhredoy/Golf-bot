/* 
 * File:   ECE312Lab3.c
 * Author: Zeeshan Haque  1753107
 *
 * Created on November 9, 2024, 1:16 PM
 */


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdarg.h>
#include "hd44780.h"
#include "lcd.h"
#include "defines.h"
#include <avr/interrupt.h>


//-------------------------
// Function used to measure the distance of the object in front of sensor
float measuring(void) {

    PORTB |= (1 << PB2);
    _delay_us(10);
    PORTB &= ~(1 << PB2);
    TIFR1 |= (1 << ICF1);
    TCCR1B |= (1 << ICES1);

    while (!(TIFR1 & (1 << ICF1)));
    uint16_t rising_edge_time = ICR1;


    TCCR1B &= ~(1 << ICES1);
    TIFR1 |= (1 << ICF1);
    while (!(TIFR1 & (1 << ICF1)));
    uint16_t falling_edge_time = ICR1;


    uint16_t time_took = falling_edge_time - rising_edge_time;
    float distance = (time_took)*0.0093045;

    return distance;

}

//Sets up LCD functionality
FILE lcd_str = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE); // to create global variable for LCD stream (before main function)

// Function for using vprintf
void lcd_vprintf(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vfprintf(&lcd_str, format, args);
    va_end(args);
}


// Setting up interrupt for the golf hole stability
volatile bool stability = false;

ISR(TIMER2_COMPA_vect) {
    if (stability) {
        PORTB |= (1 << PB3); // Turn LED ON
    } else {
        PORTB &= ~(1 << PB3); // Turn LED OFF
    }


}

// Sets PWM for the stability function
void PWM() {
    DDRD |= (1 << PD6);

    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    TCCR0A |= (1 << COM0A1);
    TCCR0B |= (1 << CS00);

    OCR0A = 0;
}

// Function to calculate the velocity of the motor arm 
uint8_t mapVelocityToPWM(float v0, float v_min, float v_max) {
    if (v0 < v_min) {
        v0 = v_min;
    }
    if (v0 > v_max) {
        v0 = v_max;
    }

    return (uint8_t) (((v0 - v_min) / (v_max - v_min)) * (240 - 200) + 200);
}

// Sets the duty cycle of PWM signal going to the motor depending on the measured sensor reading
void setMotorSpeed(float distance, float x_min, float x_max, float v_min, float v_max) {
    float v0 = ((distance - x_min) / (x_max - x_min)) * (v_max - v_min) + v_min;

    if (v0 < v_min) v0 = v_min;
    if (v0 > v_max) v0 = v_max;

    uint8_t pwmDutyCycle = mapVelocityToPWM(v0, v_min, v_max);

    OCR0A = pwmDutyCycle;
}

//Using timer 2 to set up the interrupt 
void initilizing_timer2_interrupt() {

    DDRB |= (1 << PB3);

    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    OCR2A = 156;

    TIMSK2 |= (1 << OCIE2A);
    sei();

}

int main(void) {

    initilizing_timer2_interrupt();
    PWM();
  
    // Initializes the echo pin for sensor
    DDRB &= ~(1 << PB0);
    
    //Initialize the trigger pin for sensor
    DDRB |= (1 << PB2);
    
    // Initializes the Release button 
    DDRD &= ~(1 << PD7);
    PORTD |= (1 << PD7);


    // Initializes RED LED
    DDRC |= (1 << PC1);
    PORTC &= ~(1 << PC1);

    // Initializes Green LED
    DDRB |= (1 << PB3);
    PORTC &= ~(1 << PB3);
    
    // Initialize button for Manual Mode
    DDRB |= (1 << PB1);
    PORTB |= (1 << PB1);


    // Sets y-component of joystick as an input
    DDRC &= ~(1 << PC2);
    PORTC &= ~(1 << PC2);

    
    float previous_distance = 0.0;


    while (1) {
        lcd_init();

        float dist_measure = measuring();

   
        hd44780_outcmd(HD44780_DDADDR(0x04));
        hd44780_wait_ready(false);

        fprintf(&lcd_str, "%d cm/s", OCR0A);
        
        hd44780_wait_ready(false);
     


        //If PB1 (Manual Mode Button) is pressed, Manual mode runs instead of Auto Mode
        if (!(PINB & (1 << PB1))) {


            // Selects ADC channel
            ADMUX |= (1 << REFS0); // Set ADMUX to 0x42 (ADC2 with AVCC)
            ADMUX &= ~(1 << REFS1);

            ADMUX &= ~(1 < MUX3);
            ADMUX &= ~(1 << MUX2);
            ADMUX &= ~(1 << MUX0);
            ADMUX |= (1 << MUX1);



            // Start ADC conversion
            ADCSRA |= (1 << ADSC); // Set ADSC to start conversion
            ADCSRA |= (1 << ADEN);

            //Initialize the IN1 pin on the driver module, decides direction of motor rotation
            DDRB |= (1 << PB5);
            PORTB |= (1 << PB5);



            // Wait for conversion to complete
            while (ADCSRA & (1 << ADSC)) {
                // ADSC is still set to 1, indicating conversion is in progress
                // Loop until ADSC is cleared to 0
            }

            // Read the ADC result
            uint8_t low_byte = ADCL; // Read the low byte first (ADCL)
            uint8_t high_byte = ADCH; // Then read the high byte (ADCH)
            uint16_t adc_value = ((uint16_t) high_byte << 8) | low_byte; // Combine the two bytes to form the 16-bit result

            //Scaled ADC value to map to 8 bit representation, selects min and max values
            uint8_t scaled_value = (uint8_t) ((adc_value >> 2) * (200 - 120) / 200 + 120); //135

            //Set duty cycle when in Manual Mode
            OCR0A = scaled_value; 
            lcd_vprintf("\x1b\xc0  Manual Mode");
    
        } else {
            //Auto mode is enabled when Manual mode not selected
            PORTB &= ~(1 << PB5);
            if ((dist_measure >= previous_distance - 2.0) && (dist_measure <= previous_distance + 2.0)) {
                stability = true;
                PORTC &= ~(1 << PC1);
                setMotorSpeed(dist_measure, 5.0, 120.0, 5.0, 30.0);
            } else {
                stability = false;
                PORTC |= (1 << PC1);
                PORTB &= ~(1 << PB5);
            }
            lcd_vprintf("\x1b\xc0   Auto Mode");

        }


        // Release button for the motor to spin
        if (!(PIND & (1 << PD7))) {

            DDRB |= (1 << PB5);
            PORTB |= (1 << PB5);

        } else {
            //DDRB &= ~(1 << PB5);
            PORTB &= ~(1 << PB5);
        }
        
        previous_distance = measuring();

        _delay_ms(150);
    }

    return 0;
}
