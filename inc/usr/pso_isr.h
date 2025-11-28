/*
 * pso_isr.h
 *
 *  Created on: 10/03/2014
 *      Author: Rogerio
 */

#ifndef PSO_ISR_H_
#define PSO_ISR_H_

/* Interrupt Service Routine function prototypes */
extern void UART0IntHandler(void);
extern void WTimer1AIntHandler(void);
extern void WTimer1BIntHandler(void);
extern void WTimer5AIntHandler(void);
extern void WTimer5BIntHandler(void);
extern void ADC0SS1IntHandler(void);
extern void ADC1SS1IntHandler(void);

#endif /* PSO_ISR_H_ */
