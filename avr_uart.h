/*
 * avr_uart.h
 *
 * Library for using the UART0 functionality of the ATMEGA328P.
 * This library doesn't account for overflowing of the buffers!
 * Unintended behaviour is expected when buffers are not emptied in time.
 *
 * Created: 6-3-2023 13:23:16
 * Author:  Perry Petiet
 */ 
#define F_CPU 16000000UL

#ifndef AVR_UART_H_
#define AVR_UART_H_

/******************************* INCLUDES ********************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>


/******************************* DEFINES *********************************/

#define DATA_BUFFER_LEN 32

#define PARITY_NONE 0
#define PARITY_ODD  1                         
#define PARITY_EVEN 2

/******************************* TYPEDEFS ********************************/

typedef struct
{
  uint8_t data[DATA_BUFFER_LEN];                    /* Pointer to data  */
  uint8_t head;                                     /* Head of buffer   */
  uint8_t tail;                                     /* Tail of buffer   */
  bool buffer_full;                                 /* Buffer overflow  */
} DATA_BUFFER;

typedef struct
{
  uint32_t baudRate;                                 /* Baudrate of uart */
  uint8_t  byteSize;                                 /* Byte size of uart*/
  uint8_t  parity;                                   /* Parity of uart   */
  uint8_t  stopBits;                                 /* Stop bits amount */

  DATA_BUFFER rx;                                    /* Receive buffer   */
  DATA_BUFFER tx;                                    /* Transmit buffer  */

} UART_DATA;

/******************************* LOCAL FUNCTIONS *************************/


/******************************* GLOBAL FUNCTIONS ************************/

bool init_uart(uint32_t baudRate, 
               uint8_t  byteSize, 
               uint8_t  parity, 
               uint8_t  stopBits);

bool deinit_uart();

bool uart_transmit(uint8_t* data, uint8_t data_len);

bool uart_read_byte(uint8_t* byte);


/******************************* THE END *********************************/
               
#endif /* AVR_UART_H_ */