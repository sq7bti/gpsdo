//******************************************************************************
//   USCI_A0_uart.h MSP430G2553 simple UART output
//
//   Description: Basic UART communication with 16MHz clock.
//	 Additional formating functions for easier use. RX has 16 char buffer
//
//   Musti
//   wlan slovenia http://dev.wlan-si.net
//   May 2012
//   Based on: 	TI, D.Dong MSP430G2xx3 examples
//******************************************************************************
#include  "common.h"

#ifndef USCI_A0_UART_H_
#define USCI_A0_UART_H_

//#define RX_BUFF_LENGTH 90

//                 0    1    2    3    4    5
enum frame { UNKNOWN, RMC, VTG, GGA, GSA, GSV };

//extern volatile uint8_t rxbuffer[];		//serial buffer, simple linear until break character /r
//extern volatile uint8_t rxTrack, msg_count;
extern volatile uint8_t checksum, checksum_idx;
extern volatile enum frame frame_type;
//extern volatile uint16_t* frame_counter;
extern volatile bool new_frame, crc_good;

// NMEA fields:
extern volatile char fix_status;
extern volatile bool fix_status_upd;
extern volatile uint8_t used_sats;
extern volatile bool used_sats_upd;
extern volatile char date[];
extern volatile uint8_t date_upd;
extern volatile char time[];
extern volatile uint8_t time_upd;
extern volatile char longitude[];
extern volatile uint16_t longitude_upd;
extern volatile char latitude[];
extern volatile uint16_t latitude_upd;

void initUART(void);					//init the uart interface
void putstring(const char *str);					//send string
bool txBusy();

uint8_t h2i(char h);
extern char i2h(uint8_t i);

#endif /* USCI_A0_UART_H_ */
