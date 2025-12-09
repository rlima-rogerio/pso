/*******************************************************************************
 * FILENAME:    pso_rpm.h
 *
 * DESCRIPTION:
 *       ???
 *
 * AUTHOR:      Rogerio Lima
 *              Fixed: December 2025
 ******************************************************************************/

#ifndef PSO_CONFIG_H_
#define PSO_CONFIG_H_

/*******************************************************************************
* CONFIGURATION DEFINES
 ******************************************************************************/
#define PWM_FREQUENCY           50U
#define STREAMING_RATE_HZ       125000U     /* 125 kHz data rate */
#define FIFO_BUFFER_SIZE        256U        /* FIFO buffer size */
#define ERROR_BLINK_COUNT       5U
#define FINISH_BLINK_COUNT      10U

#endif /* PSO_CONFIG_H_ */