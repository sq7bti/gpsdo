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

volatile uint8_t rxbuffer[128];		//serial buffer, simple linear until break character /r
volatile uint8_t bitTrack = 0;		//serial buffer counter
volatile uint16_t crc, crc_f;
enum frame frame_type;
volatile bool _new_frame;

const uint8_t* rxBuf() { return (const uint8_t*)rxbuffer; };
const uint8_t rxCount() { return (const uint8_t)bitTrack; }
const uint8_t checksum() { return (const uint8_t)crc_f; };
enum frame rx_frame_type() { return frame_type; }
void ack_frame() { _new_frame = FALSE; };
bool new_frame() { return _new_frame; };

void initUART(void)
{
	  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
		UCA0BR0 = 0x0C;                            // 16MHz 4800
	  UCA0BR1 = 0x0D;                            // 16MHz 4800
	  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
		IFG2 &= ~(UCA0RXIFG | UCA0RXIFG);
	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
//		__enable_interrupt();
	  __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled
}

void putc(uint8_t data){
	while (!(IFG2 & UCA0TXIFG));                // USCI_A0 TX buffer ready?
	UCA0TXBUF = data;
	IFG2 &= ~UCA0RXIFG;
}

void puts(const char *str)
{
    while(*str!=0)
       putc(*str++);
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
	uint8_t RXbyte = UCA0RXBUF;
	if(RXbyte == '$') {
		bitTrack = 0;
		crc = 0;
		frame_type = UNKNOWN;
	} else {
		if(bitTrack == 2) {
			switch(RXbyte) {
				case 'R': frame_type = RMC; break;
				case 'V': frame_type = VTG; break;
			}
		}
		if(bitTrack == 3) {
			switch(RXbyte) {
				case 'G': frame_type = GGA; break;
				case 'S': frame_type = GSA; break;
			}
		}
		crc ^= RXbyte;
		if(RXbyte == '*') {
			crc_f = crc;
			_new_frame = TRUE;
		}
		rxbuffer[bitTrack] = RXbyte;
		bitTrack++;
		bitTrack &= 0x7F; // 128 bytes buffer
		if((RXbyte == '\r') || (RXbyte == '\n')) {
			bitTrack = 0;
		}
	}
}
