#ifndef COMMON_H
#define COMMON_H 1

#include <msp430g2553.h>
/*#include <io.h>*/
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
//#include <stdio.h>
#include <stdbool.h>

#ifndef TRUE
#define TRUE true
#endif
#ifndef FALSE
#define FALSE false
#endif
#ifndef BOOL
typedef bool BOOL;
#endif

#define OVERCLOCK 20
#define CAPTURE_MULT 50000
#define USE_10MHZ_INPUT_AS_TACLK 1

#endif
