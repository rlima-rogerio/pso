#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"	/* Interrupt and register assignments on the Tiva C LauchPad board */
#include "hw_memmap.h"      /* Macros defining the memory map of the device. */
#include "hw_types.h"       /* Macros for hardware access, direct and via the bit-band region. */
#include "sysctl.h"         /* Prototypes for the system control driver. */
#include "gpio.h"           /* Defines and Macros for GPIO API */
#include "debug.h"          /* Macros for assisting debug of the driver library. */
#include "pwm.h"            /* API function protoypes for Pulse Width Modulation (PWM) ports */
#include "pin_map.h"        /* Mapping of peripherals to pins for all parts. */
#include "hw_gpio.h"        /* Defines and Macros for GPIO hardware. */
#include "rom.h"            /* Macros to facilitate calling functions in the ROM. */
#include "timer.h"          /* Prototypes for the timer module */
#include "uart.h"           /* Defines and Macros for the UART. */
#include "interrupt.h"      /* Prototypes for the NVIC Interrupt Controller Driver */
#include "pso_init.h"
#include "pso_uart.h"
#include "pso_led.h"
#include "pso_data.h"
#include "ulink.h"
#include "ulink_types.h"
#include "adc.h"
#include "pso_pwm.h"
#include "ff.h"		/* Declarations of FatFs API --- */
#include "fifo.h"

/* ff11a */
#if (FAT_FS11A)
FATFS FatFs;		/* FatFs work area needed for each volume */
FIL Fil;			/* File object needed for each open file */
#endif

/* ff09b */
#if (FAT_FS09B)
FATFS fs[1];         // Work area (file system object) for logical drives
FIL ftxt;            // file objects
BYTE buffer[16384];    // file copy buffer
FRESULT res;         // FatFs function common result code
WORD bw;             // File write count
CHAR filename[12];
#endif


#define PART_TM4C123GH6PM    /* Used for Port/Pin Mapping Definitions defined in "pin_map.h" */
#define PWM_FREQUENCY  50
#define BUFFERSIZE16KB 16384

extern uart_raw_data_t g_uart0_data;
extern uint8_t g_tx_buffer_uart;
extern uint16_t uart_tx_buffer[ULINK_MAX_PACKET_LEN];
extern uint8_t g_timer_a0_scan_flag;
extern ulink_pso_data_t dp;
extern fifo_t g_fifo_ping;
extern fifo_t g_fifo_pong;
extern uint8_t pwm_throttle;

uint16_t scan_period_actual;
uint8_t record_sd = 0U;   /* SD record flag */
uint8_t state = 0U;       /* FIFO FSM */
uint8_t fix_rpm_start_acq = 0U;       /* Flag to fix RPM and start of acq. */
bool b_aux;



int main(void)
{
	uint16_t i;
	UINT bw;
	uint8_t sys_state = 0U;
	uint8_t enable_fifo_write = 1U;
	uint8_t ret_func = 0U;
	static uint16_t k = 0U;
	uint8_t percent = 0U;

	/* 16MHz -> PLL -> 400MHz -> (1/2 * 1/5 = 1/10) = 40MHz  */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	PSO_PeripheralEnable();
	PSO_GPIOConfig();
	PSO_UART0Config();
	PSO_Timers();
	PSO_ADCConfig();
	for (i = 0U; i < 10000; i++);
	pso_rpm_config();
	pso_pwm_config();
	pso_spi0_config();

	IntMasterEnable();

	PSO_LEDWhiteOff();

/*******************************************************************************
* 1) Ajustar % do sinal PWM
* 2) Aguardar tempo para estabilizacao da RPM
* 3) Esvaziar FIFO
* 4) Fazer aquisicao
* 5) Salvar dados no SD cujo nome contenha os parametros do teste.
*
*
*******************************************************************************/

	while(1)
    {
        scan_period_actual = TIMER3_TAR_R;

		if (g_timer_a0_scan_flag)
		{
			g_timer_a0_scan_flag = 0U;

			packet_data (&dp);
			copy_data(uart_tx_buffer, &dp);

			/* Recording data to the SD card */
			if (record_sd && enable_fifo_write)
			{

/* DEBUG */     GPIO_PORTF_DATA_R |= GPIO_PIN_1; /* Red  LED on PF1 */

				res = f_write(&ftxt, &buffer, sizeof(buffer), &bw);
				// Close open file
				f_close(&ftxt);
				// Unregister work area prior to discard it
				f_mount(0, NULL);

/* DEBUG */     GPIO_PORTF_DATA_R &= ~GPIO_PIN_1; /* Red  LED on PF1 */
				record_sd = 0U;
			}

			/* System Finite State Machine (FSM) */
			switch (sys_state)
			{
			/* State 0: Waits for SW1 be pressed */
				case 0U:

					/* Switch #1 */
					if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4))
						sys_state = 1U;
					else
						sys_state = 0U;

					sd_stand_by();

					break;

			/* State 1: Starts SD card and register FatFs work area */
				case 1U:
#if (FAT_FS09B)
					sprintf(filename,"daq_%d.txt",percent);
					puts("FsFAT Testing");
                    percent++;
					memset(&fs, 0, sizeof(FATFS));

					// Register work area for each volume (Always succeeds regardless of disk status)
					res = f_mount(0, &fs[0]);

					if (res != FR_OK)
						printf("res = %d f_mount\n", res);

					// Create destination file on the drive 0
					res = f_open(&ftxt, filename, FA_CREATE_ALWAYS | FA_WRITE);

					if (res == FR_OK)
					{
						fix_rpm_start_acq = 1U;
						enable_fifo_write = 1U;
						sd_ok ();
						sys_state = 2U;
					}
					else
					{
						sys_state = 5U;
					}
#endif
					break;

			/* State 2: Starts to fill fifos and write data to the memory*/
				case 2U:
/* DEBUG */			led_blue_toggle ();

                    /* Don't change PWM/RPM until have acquired data */
                    if (~fix_rpm_start_acq)
					    ret_func = fun_trapezoid ();

                    /* Switch #2  - Finish the recording */
					if ( !(GPIO_PORTF_DATA_R & GPIO_PIN_0) || (ret_func) )
						sys_state = 3U; /* SW2 pressed: stops recording */
					else
						sys_state = 2U;  /* SW2 non pressed */
					break;

				case 3U:
					// Close open file
					f_close(&ftxt);
					// Unregister work area prior to discard it
					f_mount(0, NULL);
					enable_fifo_write = 0U; /* Disables write to the fifo */
					sys_state = 4U;
					break;

				case 4U:
					for (i = 0U; i < 10U; i++)
						sd_finish_record();     /* Turn on white LED */

					sys_state = 0U;
					break;

				case 5U: /* SD error state */
					for (i = 0U; i < 5U; i++)
						sd_error ();

					sys_state = 0U;
					break;

				default:
					/* code */
					break;

			}

			/* Fills the buffer only if RPM is fixed */
			if (k <= (BUFFERSIZE16KB - PACKET_LENGTH) && fix_rpm_start_acq )
			{
				for (i = 0U; i < PACKET_LENGTH; i++)
				{
					buffer[k] = (uint8_t)uart_tx_buffer[i];
					k++;
				}



			}
			else if (sys_state && fix_rpm_start_acq)
			{
				fix_rpm_start_acq = 0U;
				enable_fifo_write = 0U; /* Disables write to the fifo */
				sys_state = 1U;
				record_sd = 1U;
				k = 0U;
			}
			else
			{

			//	fix_rpm_start_acq = 0U;

			}



// -->
			dp.index = TIMER3_TAR_R - scan_period_actual;


			/* Switch #1 & Switch #2 pressed together*/
//				if (!(GPIO_PORTF_DATA_R & GPIO_PIN_4) & !(GPIO_PORTF_DATA_R & GPIO_PIN_0))
//				{
//					 sys_state = 0U;
//				}

		} /* END 'if' */
	} /* END 'while' */

}

