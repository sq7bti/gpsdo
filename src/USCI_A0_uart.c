//******************************************************************************
//   USCI_A0_uart.c MSP430G2553 simple UART output
//
//   Description: Basic UART communication with 16MHz clock.
//	 Additional formating functions for easier use. RX has 16 char buffer
//
//   Musti
//   wlan slovenia http://dev.wlan-si.net
//   May 2012
//   Based on: 	TI, D.Dong MSP430G2xx3 examples
//******************************************************************************
#include "USCI_A0_uart.h"
#include "printf.h"

char* volatile txTrack; // serial pointer to be transmitted
volatile uint8_t txCount; // serial pointer to be transmitted
volatile bool tx_busy = FALSE;

volatile bool rx_busy = FALSE;

volatile uint8_t rxTrack = 0;		//serial buffer counter
volatile uint8_t crc, checksum, rx_checksum, msg_count;
volatile uint8_t checksum_idx;
volatile enum frame frame_type;
volatile bool new_frame, crc_good_high, crc_good_low;
//                 0    1    2    3    4    5
//                                  { UNKNOWN, RMC, VTG, GGA, GSA, GSV };
volatile uint16_t frame_counter[] = {       0,   0,   0,   0,   0,   0 };
volatile uint16_t bad_crc_counter = 0, chars_count = 0;

#if defined RX_BUFF_LENGTH
volatile uint8_t rxbuffer[RX_BUFF_LENGTH];		//serial buffer, simple linear until break character /r
#endif /* defined RX_BUFF_LENGTH */


volatile char fix_status = 'X';
volatile bool fix_status_upd;
volatile uint8_t used_sats = 0;
volatile bool used_sats_upd = FALSE;

// 01234567
// 01:23:45
// hh mm ss
// 24-05-31
// yy mm dd
const uint8_t clock_array[] = { 0, 1, 3, 4, 6, 7 }; // 6

volatile char date[] = "dd-mm-yy";
volatile uint8_t date_upd = 0;
volatile char time[] = "hh:mm:ss";
volatile uint8_t time_upd = 0;

// latitude
// 0123456789A
// 1234.5678,N
// 12*34'5678N
// 0123456789A
//                           0  1  2  3  4  5  6  7  8  9  10
const int8_t lat_array[] = { 0, 1, 3, 4,-1, 6, 7, 8, 9,-1, 10 }; // 11
//                          0123456789A
volatile char latitude[] = "dd\x7Fmm\'ffffh"; // 12*34.5678N
volatile uint16_t latitude_upd = 0;
// 2-> degrees
// 5-> apostrophe

// longitude
// 0123456789AB
// 01234.5678,E
// 012*34'5678E
// 0124566789AA
//                           0  1  2  3  4  5  6  7  8  9 10, 11
const int8_t lon_array[] = { 0, 1, 2, 4, 5,-1, 7, 8, 9,10,-1, 11 }; // 12
//                           0123456789AB
volatile char longitude[] = "ddd\x7Fmm\'ffffh"; // 01234.5678,E
volatile uint16_t longitude_upd = 0;
// 3 -> degrees
// 4 -> apostrophe

void initUART(void)
{
	  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD

	  UCA0CTL1 |= UCSSEL_2;                     // SMCLK

// oversampling UCSO = 1
#if 0
		UCA0MCTL = UCOS16 | UCBRF_6;                // Modulation UCBRFx = 6
		// 16M/4800/16 = 208 1/3 = 0x00D0 +
		UCA0BR0 = 0xD0;                            // 208
		UCA0BR1 = 0x00;                            // 16MHz 4800
#else // oversampling UCSO = 0
// 16M/4800 = 3333 1/3 = 0x0D05
		UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	#ifdef OVERCLOCK
		UCA0BR0 = 0x46;                            // 20MHz 4800
		UCA0BR1 = 0x10;                            // 20MHz 4800
	#else
		UCA0BR0 = 0x05;                            // 16MHz 4800
		UCA0BR1 = 0x0D;                            // 16MHz 4800
	#endif /* OVERCLOCK */
#endif
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
		IFG2 &= ~(UCA0RXIFG | UCA0TXIFG);
	  IE2 |= UCA0RXIE; // | UCA0TXIE;                          // Enable USCI_A0 RX interrupt

		// enable it after all init's are finished
	  //__bis_SR_register(GIE);       // Enter LPM0, interrupts enabled
}

bool txBusy() {
	return tx_busy;
}

void putstring(const char *str)
{
	txTrack = (char* volatile)str;
	tx_busy = TRUE;
	IFG2 &= ~UCA0TXIFG;
	IE2 |= UCA0TXIE;                          // Enable USCI_A0 RX interrupt
	UCA0TXBUF = *txTrack++;
	txCount = 1;
}

//TX interrupt
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
	//__enable_interrupt();
	if(IFG2 & UCA0TXIFG) {
		if(*txTrack) {
			UCA0TXBUF = *txTrack++;
			++txCount;
		} else {
			IE2 &= ~UCA0TXIE;                          // Disable USCI_A0 RX interrupt
			tx_busy = FALSE;
		}
	}
}

//RX interrupt
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
	//__enable_interrupt();
	if(IFG2 & UCA0RXIFG) {
		rx_busy |= TRUE;
		uint8_t RXbyte = UCA0RXBUF;
		++chars_count;
		if(RXbyte == '$') {
			rxTrack = 0;
			crc = 0;
			frame_type = UNKNOWN;
			checksum_idx = 255;
			return;
		}

		if(RXbyte == 10) {
			//rxTrack = 0xFF; // special value
			++msg_count;
			new_frame = TRUE;
			++frame_counter[frame_type];
			if(!crc_good_high || !crc_good_low)
				++bad_crc_counter;
			LPM0_EXIT;
			rx_busy = FALSE;
			return;
		}

		if(rxTrack == 2) {
			switch(RXbyte) {
				// GPRMC
				case 'R': frame_type = RMC; break;
				// GPVTG
				case 'V': frame_type = VTG; break;
			}
		}
		if(rxTrack == 3) {
			switch(RXbyte) {
				// GPG_S_A or GPG_S_V
				case 'S': frame_type = GSA; break;
				// GPG_G_A
				case 'G': frame_type = GGA; break;
			}
		}
		if((frame_type == GSA) && (rxTrack == 4)) {
			switch(RXbyte) {
				// GPGS_A or GPGS_V
				case 'A': frame_type = GSA; break;
				// GPGSV
				case 'V': frame_type = GSV; break;
			}
		}

		if(frame_type == RMC) {
			switch(rxTrack) {
				case 6: case 7: case 8: case 9: case 10: case 11:
				if(time[clock_array[rxTrack - 6]] != RXbyte) {
					time[clock_array[rxTrack - 6]] = RXbyte;
					time_upd |= (1<<(clock_array[rxTrack-6]));
				};
				break;
				case 15: case 16: case 17: case 18:
				//case 19:
				case 20: case 21: case 22: case 23:
				//case 24:
				case 25:
				//if((lat_array[rxTrack - 15] != -1) && (latitude[lat_array[rxTrack - 15]] != RXbyte)) {
				if(latitude[lat_array[rxTrack - 15]] != RXbyte) {
					latitude[lat_array[rxTrack - 15]] = RXbyte;
					latitude_upd |= (1<<(lat_array[rxTrack-15]));
				};
				break;
				case 27: case 28: case 29: case 30: case 31:
				//case 32:
				case 33: case 34: case 35: case 36:
				//case 37:
				case 38:
				//if((lon_array[rxTrack - 27] != -1) && (longitude[lon_array[rxTrack - 27]] != RXbyte)) {
				if(longitude[lon_array[rxTrack - 27]] != RXbyte) {
					longitude[lon_array[rxTrack - 27]] = RXbyte;
					longitude_upd |= (1<<(lon_array[rxTrack-27]));
				};
				break;
				case 50: case 51: case 52: case 53: case 54: case 55:
				if(date[clock_array[rxTrack - 50]] != RXbyte) {
					date[clock_array[rxTrack - 50]] = RXbyte;
					date_upd |= (1<<(clock_array[rxTrack-50]));
				};
				break;
				case 13:
				if(fix_status != RXbyte) {
					fix_status = RXbyte;
					fix_status_upd = TRUE;
				}
				break;
			}
		}
		if(frame_type == GGA) {
			if(rxTrack == 40) {
				if((used_sats / 10) != h2i(RXbyte)) {
					used_sats_upd = TRUE;
					used_sats %= 10;
					used_sats += 10 * h2i(RXbyte);
				}
			}
			if(rxTrack == 41) {
				if((used_sats % 10) != h2i(RXbyte)) {
					used_sats_upd = TRUE;
					used_sats /= 10;
					used_sats *= 10;
					used_sats += h2i(RXbyte);
				}
			}
		}

		if(RXbyte == '*') {
			checksum = crc;
			checksum_idx = rxTrack + 1;
		} else
			crc ^= RXbyte;

#if defined RX_BUFF_LENGTH
		rxbuffer[rxTrack] = RXbyte;
#endif /* defined RX_BUFF_LENGTH */

		if(rxTrack == checksum_idx) {
			if(i2h(checksum >> 4) == RXbyte)
				crc_good_high = TRUE;
			else
				crc_good_high = FALSE;
		}
		if(rxTrack == (checksum_idx + 1)) {
			if(i2h(checksum & 0x0F) == RXbyte)
				crc_good_low = TRUE;
			else
				crc_good_low = FALSE;
		}

		rxTrack++;
#if defined RX_BUFF_LENGTH
	#if RX_BUFF_LENGTH == 128
		rxTrack &= 0x7F; // 128 bytes buffer
	#else
		if(rxTrack >= RX_BUFF_LENGTH)
			rxTrack = RX_BUFF_LENGTH;
	#endif /* defined RX_BUFF_LENGTH */
#else
	rxTrack &= 0x7F; // 128 bytes buffer
#endif
	}
}
