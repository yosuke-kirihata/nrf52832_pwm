/**
 * Copyright (c) 2009 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "math.h"

// Peripheral channel assignments
#define PWM0_GPIOTE_CH      0
#define PWM0_PPI_CH_A       0
#define PWM0_PPI_CH_B       1
#define PWM0_TIMER_CC_NUM   0


// TIMER3 reload value. The PWM frequency equals '16000000 / TIMER_RELOAD'
#define TIMER_RELOAD        1024
// The timer CC register used to reset the timer. Be aware that not all timers in the nRF52 have 6 CC registers.
#define TIMER_RELOAD_CC_NUM 5


// This function initializes timer 3 with the following configuration:
// 24-bit, base frequency 16 MHz, auto clear on COMPARE5 match (CC5 = TIMER_RELOAD)
void timer_init()
{
    NRF_TIMER3->BITMODE                 = TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->PRESCALER               = 0;
    NRF_TIMER3->SHORTS                  = TIMER_SHORTS_COMPARE0_CLEAR_Msk << TIMER_RELOAD_CC_NUM;
    NRF_TIMER3->MODE                    = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->CC[TIMER_RELOAD_CC_NUM] = TIMER_RELOAD;    
}


// Starts TIMER3
void timer_start()
{
    NRF_TIMER3->TASKS_START = 1;
}


// This function sets up TIMER3, the PPI and the GPIOTE modules to configure a single PWM channel
// Timer CC num, PPI channel nums and GPIOTE channel num is defined at the top of this file
void pwm0_init(uint32_t pinselect)
{  
    NRF_GPIOTE->CONFIG[PWM0_GPIOTE_CH] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos | 
                                         GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos | 
                                         pinselect << GPIOTE_CONFIG_PSEL_Pos | 
                                         GPIOTE_CONFIG_OUTINIT_High << GPIOTE_CONFIG_OUTINIT_Pos;

    NRF_PPI->CH[PWM0_PPI_CH_A].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[PWM0_TIMER_CC_NUM];
    NRF_PPI->CH[PWM0_PPI_CH_A].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[PWM0_GPIOTE_CH];
    NRF_PPI->CH[PWM0_PPI_CH_B].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[TIMER_RELOAD_CC_NUM];
    NRF_PPI->CH[PWM0_PPI_CH_B].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[PWM0_GPIOTE_CH];
    
    NRF_PPI->CHENSET               = (1 << PWM0_PPI_CH_A) | (1 << PWM0_PPI_CH_B);
}


// Function for changing the duty cycle on PWM channel 0
void pwm0_set_duty_cycle(uint32_t value)
{
    if(value == 0)
    {
        value = 1;
    }
    else if(value >= TIMER_RELOAD)
    {
        value = TIMER_RELOAD - 1;
    }
    NRF_TIMER3->CC[PWM0_TIMER_CC_NUM] = value;
}


// Utility function for providing sin values, and converting them to integers.
// input values in the range [0 - input_max] will be converted to 0-360 degrees (0-2*PI).
// output values will be scaled to the range [output_min - output_max].
uint32_t sin_scaled(uint32_t input, uint32_t input_max, uint32_t output_min, uint32_t output_max)
{
    float sin_val = sinf((float)input * 2.0f * 3.141592f / (float)input_max);
    return (uint32_t)(((sin_val + 1.0f) / 2.0f) * (float)(output_max - output_min)) + output_min; 
}


int main(void)
{
    uint32_t counter = 0;
    
    // Initialize the timer
    timer_init();
    
    // Initialize PWM channel 0
    pwm0_init(LED_1);
    
    // Start the timer
    timer_start();

    while (true)
    {
        nrf_delay_us(4000);
        
        // Update the duty cycle of PWM channel 0 and increment the counter.
        pwm0_set_duty_cycle(sin_scaled(counter++, 200, 0, TIMER_RELOAD));
    }
}