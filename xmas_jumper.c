#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define TREE1 PD2
#define TREE1T PD3
#define TREE2 PD6
#define TREE2T PD5
#define TREE3 PD0
#define TREE3T PD1
#define NOSE PD4

#define DEBOUNCE_COUNT 196
#define NUM_MODES 5

typedef struct step
{
    const uint8_t pin;
    const uint8_t state;
    const uint16_t delay;
} step_t;

typedef struct mode
{
    const uint8_t num_steps;
    const step_t steps[50];
} mode_t;


const mode_t modes[NUM_MODES] PROGMEM = {
    {
        .num_steps = 20,
        .steps = {
            { TREE1,  1, 200 },
            { TREE3,  1, 200 },
            { TREE2,  1, 200 },
            { TREE1T, 1, 200 },
            { TREE3T, 1, 200 },
            { TREE2T, 1, 800 },
            { TREE1,  0, 0 },
            { TREE2,  0, 0 },
            { TREE3,  0, 0 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 0 },
            { NOSE,   1, 100 },
            { NOSE,   0, 100 },
            { NOSE,   1, 100 },
            { NOSE,   0, 100 },
            { NOSE,   1, 100 },
            { NOSE,   0, 100 },
            { NOSE,   1, 100 },
            { NOSE,   0, 500 }
        }
    },
    {
        .num_steps = 14,
        .steps = {
            { TREE1,  1, 0 },
            { TREE2,  1, 0 },
            { TREE3,  1, 130 },
            { TREE1,  0, 0 },
            { TREE2,  0, 0 },
            { TREE3,  0, 0 },
            { TREE1T, 1, 0 },
            { TREE2T, 1, 0 },
            { TREE3T, 1, 0 },
            { NOSE,   1, 130 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 0 },
            { NOSE,   0, 0 }
        }
    },
    {
        .num_steps = 15,
        .steps = {
            { TREE1,  1, 0 },
            { TREE2,  1, 0 },
            { TREE3,  1, 100 },
            { TREE1T, 1, 10 },
            { TREE1T, 0, 100 },
            { TREE1T, 1, 15 },
            { TREE1T, 0, 150 },
            { TREE3T, 1, 5 },
            { TREE3T, 0, 80 },
            { TREE2T, 1, 10 },
            { TREE2T, 0, 130 },
            { TREE2T, 1, 5 },
            { TREE2T, 0, 50 },
            { TREE2T, 1, 10 },
            { TREE2T, 0, 150 }
        }
    },
    {
        .num_steps = 30,
        .steps = {
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { TREE1,  1, 60 },
            { TREE1T, 1, 250 },
            { TREE1,  0, 0 },
            { TREE1T, 0, 250 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { TREE2,  1, 60 },
            { TREE2T, 1, 250 },
            { TREE2,  0, 0 },
            { TREE2T, 0, 250 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { NOSE,   1, 80 },
            { NOSE,   0, 80 },
            { NOSE,   1, 80 },
            { NOSE,   0, 60 },
            { TREE3,  1, 60 },
            { TREE3T, 1, 250 },
            { TREE3,  0, 0 },
            { TREE3T, 0, 250 }
        }
    },
    {
        .num_steps = 46,
        .steps = {
            { TREE1T, 1, 0 },
            { TREE2T, 1, 0 },
            { TREE3T, 1, 250 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 750 },
            { TREE1T, 1, 0 },
            { TREE2T, 1, 0 },
            { TREE3T, 1, 100 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 500 },
            { TREE1T, 1, 0 },
            { TREE2T, 1, 0 },
            { TREE3T, 1, 800 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 500 },
            { TREE1T, 1, 0 },
            { TREE2T, 1, 0 },
            { TREE3T, 1, 0 },
            { TREE1,  1, 0 },
            { TREE2,  1, 0 },
            { TREE3,  1, 0 },
            { NOSE,   1, 350 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 0 },
            { TREE1,  0, 0 },
            { TREE2,  0, 0 },
            { TREE3,  0, 0 },
            { NOSE,   0, 200 },
            { TREE1T, 1, 0 },
            { TREE2T, 1, 0 },
            { TREE3T, 1, 0 },
            { TREE1,  1, 0 },
            { TREE2,  1, 0 },
            { TREE3,  1, 0 },
            { NOSE,   1, 500 },
            { TREE1T, 0, 0 },
            { TREE2T, 0, 0 },
            { TREE3T, 0, 0 },
            { TREE1,  0, 0 },
            { TREE2,  0, 0 },
            { TREE3,  0, 0 },
            { NOSE,   0, 1000 }
        }
    }
};


volatile uint8_t button_pressed = 0;
volatile uint8_t button_actioned = 0;


ISR(TIMER0_OVF_vect)
{
    if (bit_is_clear(PINB, PB3)) {
        if (button_pressed < DEBOUNCE_COUNT) {
	    button_pressed++;
	}
    } else {
        button_pressed = 0;
        button_actioned = 0;
    }
}


void delay_ms(int8_t ms)
{
    for (int8_t i=0; i<ms; i++) {
        _delay_ms(1);
    }
}


void do_step(uint8_t pin, uint8_t state, uint16_t delay)
{
    if (!state) {
        PORTD &= ~(1 << pin);
    } else {
        PORTD |= 1 << pin;
    }

    while (delay > 0 &&
          // We want to skip the delay if the button has been presssed
          (button_pressed < DEBOUNCE_COUNT || button_actioned)) {

        if (delay >= 100) {
            delay_ms(100);
            delay -= 100;
        } else {
            delay_ms(delay);
            delay = 0;
        }
    }
}


int main(void)
{
    /* Set up timer */
    // Normal mode
    // Divide by 8 prescaler
    TCCR0B |= (1 << CS01);
    // Overflow interrupt enable
    TIMSK |= (1 << TOIE0);

    /* Enable interrupts */
    sei();

    /* Set up button */
    // Enable PB3 as input with pull-up
    DDRB &= ~(1 << PB3);
    PORTB |= (1 << PB3);

    /* Set up output pins */
    DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6);
    PORTD = 0;

    mode_t* curr_mode;
    uint8_t curr_mode_num = 0;
    step_t* curr_step;
    uint8_t curr_step_num = 0;

    curr_mode = (mode_t*) &modes[curr_mode_num];

    while(1) {
        curr_step = (step_t*) &curr_mode->steps[curr_step_num];
        do_step(
            pgm_read_byte(&curr_step->pin),
            pgm_read_byte(&curr_step->state),
            pgm_read_word(&curr_step->delay));

        curr_step_num++;
        if (curr_step_num >= pgm_read_byte(&curr_mode->num_steps)) {
            curr_step_num = 0;
        }

	    if (button_pressed == DEBOUNCE_COUNT && !button_actioned) {
	        // Change mode
            curr_mode_num++;
	        if (curr_mode_num >= NUM_MODES) {
                curr_mode_num = 0;
            }
            curr_mode = (mode_t*) &modes[curr_mode_num];

            curr_step_num = 0;

            PORTD = 0;

            button_actioned = 1;
	    }

    }

    return 0;
}
