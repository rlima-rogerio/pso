/******************************************************************************
* FILENAME:    tracker.h
*
* DESCRIPTION:
*       Functions to...
*
* FUNCTIONS:
*    uint8_t calc_vector (gps_data_t* gps_base, gps_data_t* gps_uav);
*
* NOTES:
*       None.
*
* REFERENCES:
*       None.
*
* START DATE:    16 Aug 2015
*
* CHANGES :
*
* VERSION DATE        WHO                    DETAIL
* 1.0     16 Aug 2015 Rogerio Lima         Start-up coding.
*
* -----------------------------------------------------------------------------
* 2.0
******************************************************************************/

#ifndef FIFO_H_
#define FIFO_H_

#define MAX_FIFO_SIZE  512


typedef struct
{
	int16_t front;
	int16_t rear;
    uint8_t data[MAX_FIFO_SIZE];

}fifo_t;

void fifo_init (fifo_t* fifo);
bool fifo_put (fifo_t* fifo, uint8_t data);
bool fifo_del (fifo_t* fifo);
uint8_t fifo_get (fifo_t* fifo);
bool fifo_is_empty (fifo_t* fifo);
bool fifo_is_full (fifo_t* fifo);
void fifo_print (fifo_t* fifo);
uint16_t fifo_count_elements (fifo_t* fifo);
bool fifo_transfer (fifo_t* src, fifo_t* dst);
bool fifo_ping_pong (uint8_t* data_in, uint8_t* data_out, uint16_t size, uint8_t flag);


#endif /* FIFO_H_ */
