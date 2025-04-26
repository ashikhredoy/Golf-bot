/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * General stdiodemo define                                                                                      s
 *
 * $Id: defines.h 2186 2010-09-22 10:25:15Z aboyapati $
 */

/* CPU frequency */

#define F_CPU 14745600UL

/* UART baud rate */
#define UART_BAUD  384000

/* HD44780 LCD port connections */
#define HD44780_RS C, 3
#define HD44780_RW C, 4
#define HD44780_E  C, 5
/* The data bits have to be not only in ascending order but also consecutive. */



//#define HD44780_D0 D, 4
#define HD44780_D4 D, 2
#define HD44780_D5 D, 3
#define HD44780_D6 D, 4
#define HD44780_D7 D, 5

/* Whether to read the busy flag, or fall back to
   worst-time delays. */
#define USE_BUSY_BIT 1
