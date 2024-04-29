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
#include  "USCI_A0_uart.h"

char* volatile txTrack; // serial pointer to be transmitted
volatile uint8_t txCount; // serial pointer to be transmitted
volatile bool tx_busy = FALSE;

volatile uint8_t rxTrack = 0;		//serial buffer counter
volatile uint8_t crc, checksum, rx_checksum, msg_count;
volatile uint8_t checksum_idx;
volatile enum frame frame_type;
volatile bool new_frame, crc_good;
//                 0    1    2    3    4    5
//                                  { UNKNOWN, RMC, VTG, GGA, GSA, GSV };
volatile uint16_t frame_counter[] = {       0,   0,   0,   0,   0,   0 };
volatile uint16_t bad_crc_counter = 0, chars_count = 0;

volatile uint8_t rxbuffer[RX_BUFF_LENGTH];		//serial buffer, simple linear until break character /r

volatile char fix_status = 'X';
volatile bool fix_status_upd;
volatile uint8_t used_sats = 0;
volatile bool used_sats_upd = FALSE;
volatile char date[] = "yymmdd";
volatile uint8_t date_upd = 0;
volatile char time[] = "hhmmss";
volatile uint8_t time_upd = 0;
volatile char latitude[] = "ddmm_ffff_h"; // 1234.5678,N
volatile uint16_t latitude_upd = 0;
volatile char longitude[] = "dddmm_ffff_h"; // 01234.5678,E
volatile uint16_t longitude_upd = 0;

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
		UCA0BR0 = 0x05;                            // 16MHz 4800
		UCA0BR1 = 0x0D;                            // 16MHz 4800
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
	txCount = 0;
	tx_busy = TRUE;
	IFG2 &= ~UCA0TXIFG;
	IE2 |= UCA0TXIE;                          // Enable USCI_A0 RX interrupt
	UCA0TXBUF = *txTrack++;
}

// dangerous : no checks
uint8_t h2i(char h) {
	return (h > '9')?(h - 'A'):(h - '0');
};

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
	if(IFG2 & UCA0RXIFG) {
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
			if(!crc_good)
				++bad_crc_counter;
			LPM0_EXIT;
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
		if((frame_type == UNKNOWN) && (rxTrack == 4)) {
			switch(RXbyte) {
				// GPGSA or GPGGA
				case 'A': frame_type = (rxbuffer[3] == 'S')?GSA:GGA; chars_count = 5; break;
				// GPGSV
				case 'V': frame_type = GSV; break;
			}
		}

		if(frame_type == RMC) {
			switch(rxTrack) {
				case 6: case 7: case 8: case 9: case 10: case 11:
				if(time[rxTrack - 6] != RXbyte) {
					time_upd |= (1<<(rxTrack-6));
					time[rxTrack - 6] = RXbyte;
				};
				break;
				case 15: case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25:
				if(latitude[rxTrack - 15] != RXbyte) {
					latitude_upd |= (1<<(rxTrack-15));
					latitude[rxTrack - 15] = RXbyte;
				};
				break;
				case 27: case 28: case 29: case 30: case 31: case 32: case 33: case 34: case 35: case 36: case 37: case 38:
				if(longitude[rxTrack - 27] != RXbyte) {
					longitude_upd |= (1<<(rxTrack-27));
					longitude[rxTrack - 27] = RXbyte;
				};
				break;
				case 50: case 51: case 52: case 53: case 54: case 55:
				if(date[rxTrack - 50] != RXbyte) {
					date_upd |= (1<<(rxTrack-50));
					date[rxTrack - 50] = RXbyte;
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
			if(rxTrack == 41) {
				uint8_t new_used_sats = h2i(rxbuffer[40]) * 10 + h2i(RXbyte);
				if(new_used_sats != used_sats) {
					used_sats_upd = TRUE;
					used_sats = new_used_sats;
				}
			}
		}

		if(RXbyte == '*') {
			checksum = crc;
			checksum_idx = rxTrack + 2;
		} else
			crc ^= RXbyte;

		rxbuffer[rxTrack] = RXbyte;

		if(rxTrack == checksum_idx) {
			if((i2h(checksum >> 4) == rxbuffer[checksum_idx-1])
			&& (i2h(checksum & 0x0F) == rxbuffer[checksum_idx]))
				crc_good = TRUE;
			else
				crc_good = FALSE;
		}

		rxTrack++;
#if RX_BUFF_LENGTH == 128
		rxTrack &= 0x7F; // 128 bytes buffer
#else
		if(rxTrack >= RX_BUFF_LENGTH)
			rxTrack = RX_BUFF_LENGTH;
#endif
	}
}
