/******************************************************************************
 *                          Reusable MSP430 printf()
 *
 * Description: This printf function was written by oPossum and originally
 *              posted on the 43oh.com forums. For more information on this
 *              code, please see the link below.
 *
 *              http://www.43oh.com/forum/viewtopic.php?f=10&t=1732
 *
 *              A big thanks to oPossum for sharing such great code!
 *
 * Author:  oPossum
 * Source:  http://www.43oh.com/forum/viewtopic.php?f=10&t=1732
 * Date:    10-17-11
 *
 * Note: This comment section was written by Nicholas J. Conn on 06-07-2012
 *       for use on NJC's MSP430 LaunchPad Blog.
 ******************************************************************************/

#include <stdarg.h>
#include <string.h>
#include "printf.h"

//#define RECURSIVE 1

#ifndef RECURSIVE
const unsigned long dv[] = {
//  4294967296      // 32 bit unsigned max
		1000000000,// +0
		100000000, // +1
		10000000, // +2
		1000000, // +3
		100000, // +4
//       65535      // 16 bit unsigned max
		10000, // +5
		1000, // +6
		100, // +7
		10, // +8
		1, // +9
		};
#endif /* RECURSIVE */

#ifdef RECURSIVE
void xtoa(char *p[], unsigned long x) {
#else
void xtoa(char *p[], unsigned long x, const unsigned long *dp) {
#endif /* RECURSIVE */
#ifdef RECURSIVE
	if(!(**p))
		return;
	if(x < 10) {
		**p = '0' + x;
	} else {
		xtoa(p, x/10);
		if(!(**p))
			return;
		**p = '0' + (x%10);
	}
	++(*p);
#else
	char c;
	unsigned long d;
	if (x) {
		while (x < *dp)
			++dp;
		do {
			d = *dp++;
			c = '0';
			while (x >= d)
				++c, x -= d;
			**p = c;
			++(*p);
		} while (!(d & 1) && **p);
	} else {
		**p = '0';
		++(*p);
	}
#endif /* RECURSIVE */

}

// dangerous : no checks
uint8_t h2i(char h) {
	return (h > '9')?(h - 'A'):(h - '0');
};

char i2h(uint8_t i) {
  return "0123456789ABCDEF"[i&0x0F];
};


void puth(char *p[], unsigned n) {
	**p = i2h(n);
	++(*p);
}

void strprintf(char *p[], char *format, ...)
{
	char c;
	int i;
	long n;

	va_list a;
	va_start(a, format);
	while(c = *format++) {
		if(c == '%') {
			switch(c = *format++) {
				case 's': // String
					strcpy(*p, va_arg(a, char*));
					(*p) += strlen(va_arg(a, char*));
					break;
				case 'c':// Char
					**p = va_arg(a, int);
					++(*p);
					break;
				case 'i':// 16 bit Integer
				case 'u':// 16 bit Unsigned
					i = va_arg(a, int);
					if(c == 'i' && i < 0) {
						i = -i;
						**p = '-';
						++(*p);
					}
					xtoa(p, (unsigned)i
#ifndef RECURSIVE
										, dv + 5
#endif /* RECURSIVE */
												);
				break;
				case 'q':// Q8R8
					i = va_arg(a, int);
					if(c == 'i' && i < 0) {
						i = -i;
						**p = '-';
						++(*p);
					}
					xtoa(p, (unsigned)(i >> 8)
#ifndef RECURSIVE
										, dv + 5
#endif /* RECURSIVE */
												);
					**p = '.';
					++(*p);
					i &= 0x00FF;
					i *= 10;
					**p = '0' + ((i & 0xFF00) >> 8);
					++(*p);
					i &= 0x00FF;
					i *= 10;
					**p = '0' + ((i & 0xFF00) >> 8);
					++(*p);
				break;
				case 'r':// Q4R12
					i = va_arg(a, int);
					if(c == 'i' && i < 0) {
						i = -i;
						//putc('-');
						**p = '-';
						++(*p);
					}
					char decimal_places = ((i>>12)>9)?2:3;
					xtoa(p, (unsigned)(i >> 12)
#ifndef RECURSIVE
										, dv + 5
#endif /* RECURSIVE */
												);
					**p = '.';
					++(*p);
					i &= 0x0FFF;
					i *= 10;
					**p = '0' + ((i & 0xF000) >> 12);
					++(*p);
					i &= 0x0FFF;
					i *= 10;
					**p = '0' + ((i & 0xF000) >> 12);
					++(*p);
					if(decimal_places > 2) {
						i &= 0x0FFF;
						i *= 10;
						**p = '0' + ((i & 0xF000) >> 12);
						++(*p);
					}
				break;
				case 'l':// 32 bit Long
				case 'n':// 32 bit uNsigned loNg
					n = va_arg(a, long);
					if(c == 'l' && n < 0) {
						n = -n;
						**p = '-';
						++(*p);
					}
					xtoa(p, (unsigned long)n
#ifndef RECURSIVE
											, dv
#endif /* RECURSIVE */
												);
				break;
				case 'x':// 16 bit heXadecimal
					i = va_arg(a, int);
					puth(p, i >> 12);
					puth(p, i >> 8);
					puth(p, i >> 4);
					puth(p, i);
				break;
				case 0: return;
				default:
					goto bad_fmt;
			}
		} else
			bad_fmt: {
				**p = c;
				++(*p);
			}
	}
	va_end(a);
}
