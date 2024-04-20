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

volatile uint8_t bitTrack = 0;		//serial buffer counter
volatile uint8_t crc, checksum, rx_checksum, msg_count;
volatile uint8_t checksum_idx;
volatile enum frame frame_type;
volatile bool new_frame, crc_good;

volatile uint8_t rxbuffer[128];		//serial buffer, simple linear until break character /r

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
		UCA0BR0 = 0x0C;                            // 16MHz 4800
	  UCA0BR1 = 0x0D;                            // 16MHz 4800
	  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//		IFG2 &= ~(UCA0TXIFG | UCA0RXIFG);
	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
//		__enable_interrupt();
	  __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled
}

void sendchar(uint8_t data){
	while (!(IFG2 & UCA0TXIFG));                // USCI_A0 TX buffer ready?
	UCA0TXBUF = data;
	IFG2 &= ~UCA0TXIFG;
}

void putstring(const char *str)
{
    while(*str!=0)
       sendchar(*str++);
}

// dangerous : no checks
uint8_t h2i(char h) {
	return (h > '9')?(h - 'A'):(h - '0');
};

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
		if(RXbyte == '$') {
			bitTrack = 0;
			crc = 0;
			frame_type = UNKNOWN;
			return;
		}

		if(RXbyte == 10) {
			bitTrack = 0xFF; // special value
			++msg_count;
			new_frame = TRUE;
			return;
		}

		if(bitTrack == 2) {
			switch(RXbyte) {
				// GPRMC
				case 'R': frame_type = RMC; break;
				// GPVTG
				case 'V': frame_type = VTG; break;
			}
		}
		if((frame_type == UNKNOWN) && (bitTrack == 4)) {
			switch(RXbyte) {
				// GPGSA or GPGGA
				case 'A': frame_type = (rxbuffer[3] == 'S')?GSA:GGA; break;
				// GPGSV
				case 'V': frame_type = GSV; break;
			}
		}

		if(frame_type == RMC) {
			switch(bitTrack) {
				case 6: case 7: case 8: case 9: case 10: case 11:
				if(time[bitTrack - 6] != RXbyte) {
					time_upd |= (1<<(bitTrack-6));
					time[bitTrack - 6] = RXbyte;
				};
				break;
				case 15: case 16: case 17: case 18: case 19: case 20: case 21: case 22: case 23: case 24: case 25:
				if(latitude[bitTrack - 15] != RXbyte) {
					latitude_upd |= (1<<(bitTrack-15));
					latitude[bitTrack - 15] = RXbyte;
				};
				break;
				case 27: case 28: case 29: case 30: case 31: case 32: case 33: case 34: case 35: case 36: case 37: case 38:
				if(longitude[bitTrack - 27] != RXbyte) {
					longitude_upd |= (1<<(bitTrack-27));
					longitude[bitTrack - 27] = RXbyte;
				};
				break;
				case 50: case 51: case 52: case 53: case 54: case 55:
				if(date[bitTrack - 50] != RXbyte) {
					date_upd |= (1<<(bitTrack-50));
					date[bitTrack - 50] = RXbyte;
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
			if(bitTrack == 41) {
				uint8_t new_used_sats = h2i(rxbuffer[40]) * 10 + h2i(RXbyte);
				if(new_used_sats != used_sats) {
					used_sats_upd = TRUE;
					used_sats = new_used_sats;
				}
			}
		}

		if(RXbyte == '*') {
			checksum = crc;
			checksum_idx = bitTrack + 2;
		} else
			crc ^= RXbyte;

		rxbuffer[bitTrack] = RXbyte;

		bitTrack++;
		bitTrack &= 0x7F; // 128 bytes buffer
		//if(bitTrack>85)
		// 	bitTrack = 85; // 128 bytes buffer
	//	if((RXbyte == '\r') || (RXbyte == '\n')) {

		if(bitTrack == checksum_idx) {
	//		new_frame = TRUE;
			//if(checksum == (((16 * h2i(rxbuffer[checksum_idx-1])) + h2i(rxbuffer[checksum_idx]))))
			if((i2h(checksum >> 4) == rxbuffer[checksum_idx-1])
			&& (i2h(checksum & 0x0F) == rxbuffer[checksum_idx]))
				crc_good = TRUE;
			else
				crc_good = FALSE;
		}
		//__bis_SR_register_on_exit(LPM3_bits);
	}
}
