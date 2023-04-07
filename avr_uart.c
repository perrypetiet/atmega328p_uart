/*
 * avr_uart.c
 *
 * Library for using the UART0 functionality of the ATMEGA328P.
 * This library doesn't account for overflowing of the buffers!
 * Unintended behaviour is expected when buffers are not emptied in time.
 *
 * Created: 6-3-2023 13:23:16
 * Author:  Perry Petiet
 */ 
/******************************* INCLUDES ********************************/

#include "avr_uart.h"
#include <avr/interrupt.h>

/******************************* GLOBAL VARIABLES ************************/

UART_DATA* uart_data = NULL;

/******************************* LOCAL FUNCTIONS *************************/

ISR(USART_UDRE_vect)
{
  // Check if we have more data to send:
  if (uart_data->tx.tail != uart_data->tx.head)
  {
    uint8_t tail = uart_data->tx.tail;
    //increase tail
    if (uart_data->tx.tail == DATA_BUFFER_LEN - 1)
    {
      uart_data->tx.tail = 0;
    } else 
    {
      uart_data->tx.tail++;
    }
    UDR0 = uart_data->tx.data[tail];
  }
  else
  {
    UCSR0B &= ~(1 << UDRIE0);  
  }
}

ISR(USART_RX_vect)
{
  if (uart_data != NULL)
  {
    uart_data->rx.data[uart_data->rx.head] = UDR0;
    if (uart_data->rx.head == DATA_BUFFER_LEN - 1)
    {
      uart_data->rx.head = 0;
    }
    else 
    {
      uart_data->rx.head++;
    }
  }
}

/******************************* GLOBAL FUNCTIONS ************************/

bool init_uart(uint32_t baudRate, 
               uint8_t  byteSize, 
               uint8_t  parity, 
               uint8_t  stopBits)
{
  bool init_succes = false;
  if (uart_data == NULL)
  {
    if ((parity == 0 || parity == 1)     &&
        (byteSize >= 5 && byteSize <= 8) &&   
        (stopBits == 1 || stopBits == 2))
    {
      uart_data = malloc(sizeof(UART_DATA)); 
      uart_data->baudRate = baudRate;
      uart_data->byteSize = byteSize;
      uart_data->parity   = parity;
      uart_data->stopBits = stopBits;
      //Set all register to 0;

      //set to async mode
      UCSR0B |= (1 << UMSEL00 | 1 << UMSEL01);

      //set baudrate
      uint16_t ubrrRegister = F_CPU / 16 / uart_data->baudRate - 1;
      UBRR0H = (unsigned char) (ubrrRegister>>8);
      UBRR0L = (unsigned char)  ubrrRegister;

      //set bytesize
      switch(uart_data->byteSize)
      {
        case 5:
          UCSR0B &= ~(1 << UCSZ02);
          UCSR0C &= ~(1 << UCSZ01 | 1 << UCSZ00);
          break;
        case 6:
          UCSR0B &= ~(1 << UCSZ02);
          UCSR0C &= ~(1 << UCSZ01);
          UCSR0C |= (1 << UCSZ00);
          break;
        case 7:
          UCSR0B &= ~(1 << UCSZ02);
          UCSR0C |= (1 << UCSZ01);
          UCSR0C &= ~(1 << UCSZ00);
          break;
        case 8:
          UCSR0B &= ~(1 << UCSZ02);
          UCSR0C |= (1 << UCSZ01 | 1 << UCSZ00);
          break;
      }

      //set parity
      switch(uart_data->parity)
      {
        case PARITY_NONE:
          UCSR0C &= ~(1 << UPM00 | 1 << UPM01);
          break;
        case PARITY_EVEN: 
          UCSR0C &= ~(1 << UPM00);
          UCSR0C |= (1 << UPM01);
          break;
        case PARITY_ODD:
          UCSR0C |= (1 << UPM01 | 1 << UPM00);
          break;
      }

      //set stopbits
      switch(uart_data->stopBits)
      {
        case 1:
          UCSR0C &= ~(1 << USBS0);
          break;
        case 2:
          UCSR0C |= (1 << USBS0);
          break;
      }
      //eneble receiver and transmitter
      UCSR0B |= (1 << TXEN0 | 1 << RXEN0 | 1 << RXCIE0);
      UCSR0B &= ~(1 << TXCIE0);

      uart_data->rx.head = 0;
      uart_data->rx.tail = 0;
      uart_data->tx.head = 0;
      uart_data->tx.tail = 0;

      uart_data->rx.buffer_full = false;
      uart_data->tx.buffer_full = false;

      init_succes = true;
    }
  }
  return init_succes;
}

bool deinit_uart()
{
  if(uart_data != NULL)
  {
    free(uart_data);
    uart_data = NULL;
    return true;
  }
  return false;
}



bool uart_transmit(uint8_t* data, uint8_t data_len)
{
  bool transmit_success = false;

  if (uart_data != NULL)
  {
    if (data_len < DATA_BUFFER_LEN - uart_data->tx.head)
    {
      memcpy(&uart_data->tx.data[uart_data->tx.head], data, data_len);
      uart_data->tx.head += data_len;
    } else
    {
      uint8_t overflow = data_len - (DATA_BUFFER_LEN - uart_data->tx.head);

      memcpy(&uart_data->tx.data[uart_data->tx.head], 
             data, 
             data_len - overflow);
      memcpy(&uart_data->tx.data[0], 
             &data[data_len - overflow], 
             overflow);
      uart_data->tx.head = overflow;
    }

    uint8_t tail = uart_data->tx.tail;
    if (uart_data->tx.tail == DATA_BUFFER_LEN - 1)
    {
      uart_data->tx.tail = 0;
    }
    else
    {
      uart_data->tx.tail++;
    }
    UDR0 = uart_data->tx.data[tail];
    UCSR0B |= (1 << UDRIE0);
  }
  return transmit_success;
}

bool uart_read_byte(uint8_t* byte)
{
  if (uart_data != NULL)
  {
    if (uart_data->rx.tail != uart_data->rx.head)
    {
      *byte = uart_data->rx.data[uart_data->rx.tail];
      if (uart_data->rx.tail == DATA_BUFFER_LEN - 1)
      {
        uart_data->rx.tail = 0;
      }
      else 
      {
        uart_data->rx.tail++;
      }
      return true;
    }
  }
  return false;
}

/******************************* THE END *********************************/
