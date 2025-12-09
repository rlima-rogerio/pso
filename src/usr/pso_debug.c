#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "sysctl.h"
#include "gpio.h"
#include "debug.h"
#include "pwm.h"
#include "pin_map.h"
#include "hw_gpio.h"
#include "pso_data.h"
#include "pso_debug.h"


/*******************************************************************************
 * DEBUG GPIO INITIALIZATION
 ******************************************************************************/
/*******************************************************************************
 * DEBUG GPIO INITIALIZATION (coerente com PSO_GPIOConfig)
 ******************************************************************************/
void debug_gpio_init(void)
{
    /* 1. Habilitar clock para GPIO Port D */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOD;  /* Enable clock for Port D */
    
    /* 2. Pequeno delay para clock estabilizar */
    volatile uint32_t delay = SYSCTL_RCGCGPIO_R;
    
    /* 3. Unlock PD7 se necessário (PD7 tem commit control no TM4C123) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0xF0;  /* Allow changes to PD7-PD4 */
    
    /* 4. Configurar direção (output para PD4-PD7) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DIR) |= 0xF0;  /* PD7-PD4 as outputs */
    
    /* 5. Sem função alternativa */
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0xF0;
    
    /* 6. Drive strength 2mA */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DR2R) |= 0xF0;
    
    /* 7. Sem pull-up/pull-down (output não precisa) */
    HWREG(GPIO_PORTD_BASE + GPIO_O_PUR) &= ~0xF0;
    HWREG(GPIO_PORTD_BASE + GPIO_O_PDR) &= ~0xF0;
    
    /* 8. Habilitar digital */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0xF0;
    
    /* 9. Desabilitar analog */
    HWREG(GPIO_PORTD_BASE + GPIO_O_AMSEL) &= ~0xF0;
    
    /* 10. Limpar PCTL para GPIO */
    HWREG(GPIO_PORTD_BASE + GPIO_O_PCTL) &= ~0xFFFF0000;
    
    /* 11. Lock novamente */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    
    /* 12. Inicializar todos em LOW */
    HWREG(GPIO_PORTD_BASE + GPIO_O_DATA) &= ~0xF0;
    
    // /* DEBUG message */
    // const char *msg = "Debug GPIO PD4-PD7 initialized\r\n";
    // while (*msg) UARTCharPut(UART0_BASE, *msg++);
}


 // void debug_gpio_init(void)
// {
//     /* Enable GPIO Port D peripheral */
//     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    
//     /* Wait for peripheral to be ready */
//     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    
//     /* Configure debug pins as outputs */
//     GPIOPinTypeGPIOOutput(DEBUG_GPIO_PORT, 
//                          DEBUG_TIMING_STREAM | 
//                          DEBUG_TIMING_LOOP | 
//                          DEBUG_TIMING_ADC | 
//                          DEBUG_TIMING_STATE);
    
//     /* Set all debug pins low initially */
//     GPIOPinWrite(DEBUG_GPIO_PORT, 
//                 DEBUG_TIMING_STREAM | DEBUG_TIMING_LOOP | 
//                 DEBUG_TIMING_ADC | DEBUG_TIMING_STATE, 
//                 0);
// }

/*******************************************************************************
 * TIMING PULSE GENERATION
 * Generates a short pulse (1-2μs) on specified debug pin
 ******************************************************************************/
void debug_timing_pulse(uint8_t debug_pin)
{
    /* Set pin high */
    GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, debug_pin);
    
    /* Short delay (approx 1-2 CPU cycles) */
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    
    /* Set pin low */
    GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, 0);
}

/*******************************************************************************
 * TIMING MEASUREMENT START/STOP
 * Use with oscilloscope to measure execution time
 ******************************************************************************/
void debug_timing_measure(uint8_t debug_pin, uint32_t *start_time)
{
    if (start_time)
    {
        /* Start measurement - set pin high */
        GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, debug_pin);
        *start_time = get_systick_ms();
    }
    else
    {
        /* Stop measurement - set pin low */
        GPIOPinWrite(DEBUG_GPIO_PORT, debug_pin, 0);
    }
}