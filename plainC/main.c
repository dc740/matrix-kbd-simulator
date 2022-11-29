/***
* Reimplementation of EXPS2, but for a Sony HB 10p and 20p
*
* Copyright (C) 2022 Emilio Moretti
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//#define DEBUGMODE
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <util/delay.h>

#ifdef DEBUGMODE
// Debug software serial port (tx only)
#define DDRUART        DDRB
#define PORTUART       PORTB
#define BITUART        4
#define BIT_DLY_9600   103
#define BIT_DLY_19200  51
#define BIT_DLY_38400  25
#define BIT_DLY_57600  17
#define BIT_DLY_115200 8
#define BIT_DELAY      BIT_DLY_9600
#endif

#include "msxKeyboardMatrix.h"
#include "layoutFULL.h"
#include "myPS2.h"             // pin definition on the file

/// Teensy setting
#if F_CPU == 16000000L
#define ADC_PRESCALER 0x07
#define CPU_PRESCALER 0x00
#elif F_CPU == 8000000L
#define ADC_PRESCALER 0x06
#define CPU_PRESCALER 0x01
#elif F_CPU == 4000000L
#define ADC_PRESCALER 0x05
#define CPU_PRESCALER 0x02
#elif F_CPU == 2000000L
#define ADC_PRESCALER 0x04
#define CPU_PRESCALER 0x03
#elif F_CPU == 1000000L
#define ADC_PRESCALER 0x03
#define CPU_PRESCALER 0x04
#else
#error "Teensy board only supports 16, 8, 4, 2, 1 MHz.  Check your settings"
#endif

//       _      __ _      _ _   _
//    __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
//   / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
//   \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
//
void setColumn(uint8_t column, uint8_t event);
uint8_t* getColumn(uint8_t column);
void updateMatrix(uint8_t k);
void clearMatrix( void );
void poweroff();
bool initalizeKeyboard(void);
void toggle_instant_mode(void);
void setup(void);
void loop(void);
#ifdef DEBUGMODE
void printBin ( uint8_t l);
void printHex ( uint8_t l);
void softSendByte (uint8_t dado);
void debug( char *s);
void initsoftSend(void);
#endif

//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//
uint8_t COLUMN_STATUS_0 = 255;
uint8_t COLUMN_STATUS_1 = 255;
uint8_t COLUMN_STATUS_2 = 255;
uint8_t COLUMN_STATUS_3 = 255;
uint8_t COLUMN_STATUS_4 = 255;
uint8_t COLUMN_STATUS_5 = 255;
uint8_t COLUMN_STATUS_6 = 255;
uint8_t COLUMN_STATUS_7 = 255;
uint8_t COLUMN_STATUS_8 = 255;
bool EXT = false;
bool BRK = false;

//    _     _                         _
//   (_)_ _| |_ ___ _ _ _ _ _  _ _ __| |_ ___
//   | | ' \  _/ -_) '_| '_| || | '_ \  _(_-<
//   |_|_||_\__\___|_| |_|  \_,_| .__/\__/__/
//                              |_|
volatile uint8_t zero = 0;

// COLUMN_STATUS[0]
ISR (INT0_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PIND],0\n\t" //test if PORTD 0 is high or low
    "rjmp .int0_raising_edge\n\t"
    ".int0_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_0)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_0)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int0_exit_isr\n\t"
    ".int0_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int0_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int0_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int0_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[1]
ISR (INT1_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PIND],1\n\t" //test if PORTD 0 is high or low
    "rjmp .int1_raising_edge\n\t"
    ".int1_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_1)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_1)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int1_exit_isr\n\t"
    ".int1_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int1_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int1_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int1_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[2]
ISR (INT2_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PIND],2\n\t" //test if PORTD 0 is high or low
    "rjmp .int2_raising_edge\n\t"
    ".int2_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_2)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_2)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int2_exit_isr\n\t"
    ".int2_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int2_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int2_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int2_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[3]
ISR (INT3_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PIND],3\n\t" //test if PORTD 0 is high or low
    "rjmp .int3_raising_edge\n\t"
    ".int3_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_3)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_3)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int3_exit_isr\n\t"
    ".int3_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int3_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int3_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int3_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[4]
ISR (INT4_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PINE],4\n\t" //test if PORTD 0 is high or low
    "rjmp .int4_raising_edge\n\t"
    ".int4_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_4)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_4)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int4_exit_isr\n\t"
    ".int4_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int4_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int4_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int4_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[5]
ISR (INT5_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PINE],5\n\t" //test if PORTD 0 is high or low
    "rjmp .int5_raising_edge\n\t"
    ".int5_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_5)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_5)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int5_exit_isr\n\t"
    ".int5_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int5_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int5_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int5_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[6]
ISR (INT6_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PINE],6\n\t" //test if PORTD 0 is high or low
    "rjmp .int6_raising_edge\n\t"
    ".int6_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_6)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_6)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int6_exit_isr\n\t"
    ".int6_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int6_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int6_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int6_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

// COLUMN_STATUS[7]
ISR (INT7_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    "sbic %[_PINE],7\n\t" //test if PORTD 0 is high or low
    "rjmp .int7_raising_edge\n\t"
    ".int7_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_7)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_7)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .int7_exit_isr\n\t"
    ".int7_raising_edge:\n\t"
    // We need to set the signal back to 0xFF if no other column is ON (LOW really)
    "sbis %[_PINB],0\n\t" //skip exit if B0 is HIGH
    "rjmp .int7_exit_isr\n\t"
    "in	r26,%[_PIND]\n\t" //all Y0..Y7 are D0..3 and E4..7
    "in	r27,%[_PINE]\n\t"
    "or r26,r27\n\t"      // D4..7 and E0..3 are always low so we can do this
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .int7_exit_isr\n\t"
    //if another column is NOT all HIGH, then itś being tested. we exit
    //If INT0.7 (D0..3 E4..&) AND B0 are HIGH, this column is alone and since we are in the raising edge
    // then it's also done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues with whichever column was tested last (carts do whatever they want)
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".int7_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_EIFR],1 \n\t"   // reset interrupt bit.
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_EIFR]  "I" (_SFR_IO_ADDR(EIFR) )
    );
};

/*
  ISR (PCINT0_vect) { Connected to PB0 as a change bit because we are out of external level interrupts
  DDRC = COLUMN_status [8];
  }
*/
ISR (PCINT0_vect, ISR_NAKED) {
  asm volatile (
    "in __zero_reg__,__SREG__ \n\t"
    "out %[_GPIOR1],r26\n\t"
    "out %[_GPIOR2],r27\n\t"
    //Since this is a pin change interrupt, we have to check if it's falling or raising
    // so we can exit on the raising case. We only care about falling edges for our application.
    //option 1: manually reading. Thatś 1 cycle for the in + the next instructions, both cases 2 on the
    // critical path, so thatś 3 cycles to check the port bit
   //"in	r26,%[_PINB]\n\t" //First we read the port status
   // The critical case is PINB[0] == 0, so it was a falling interrupt and we need to execute it.
   // andi + brneq = For the critical case, branching does not occur. So 1 + 1 cycles
   //"andi r26,0x1\n\t"        // test if it's HIGH (raising interrupt). Sets Z if PINB[0] == 0
   //"brneq .exit_isr\n\t"     //branch if Z flag is unset, which means PINB[0] == 1 and this is a raising interrupt
   // Another alternative:
   // sbrs + rjmp = 1+1 cycles on the critical scenario
   //"sbrs r26,1\n\t"  //Skip next jump if bit 1 is HIGH (in this case 2 cycles to skip because rjmp is 1 word)
   //"rjmp .exit_isr\n\t" //2 cycles
   //option 2: rjmp is 1 cycle, so that's 2 on the critical path
   "sbic %[_PINB],0\n\t"
   "rjmp .pcint0_raising_edge\n\t"
   //now we go back to the usual ISR that a normal interrupt uses
   // Falling edge
    ".pcint0_falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_8)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_8)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .pcint0_exit_isr\n\t"
    ".pcint0_raising_edge:\n\t"
    // now we set the signal back to 0xFF IF no other column is ON (LOW really)
    "in	r26,%[_PIND]\n\t" //all Y0..Y3 are located on PORTD
    "in	r27,%[_PINE]\n\t" //all Y4..Y7 are located on PORTE
    //"andi r26,0x0F\n\t" //keep Y0..Y3
    //"andi r27,0xF0\n\t" //keep Y4..Y7
    "or r26,r27\n\t"
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .pcint0_exit_isr\n\t" //if Y4...Y7 is NOT all HIGH, we exit
    //If D0..D3 AND E4..E7 are HIGH, PB0 is alone and done
    // so we must switch PORTC back to high (remember we are in a raising edge)
    // because we are no longer being tested on this column
    // and keeping it LOW causes issues to all Y8 column
    /*
     * another way to do the same, just checking for D0
    //if pin D0 is set (OFF), reset PORTC, else jmp exit
    "sbis %[_PIND],0\n\t"
    "rjmp .exit_isr\n\t"
    "ldi r26,0xFF\n\t"*/
    "out %[_PORTC] ,r26 \n\t" //r26 is 0xFF
    ".pcint0_exit_isr:"
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"
    "sbi %[_PCIFR],1 \n\t"           // reset interrupt bit
    "out __SREG__,__zero_reg__ \n\t"
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"
    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
    [_PIND]   "I" (_SFR_IO_ADDR(PIND)  ),
    [_PINE]   "I" (_SFR_IO_ADDR(PINE)  ),
    [_PINB]   "I" (_SFR_IO_ADDR(PINB)  ),
    [_GPIOR1] "I" (_SFR_IO_ADDR(GPIOR1)),
    [_GPIOR2] "I" (_SFR_IO_ADDR(GPIOR2)),
    [_PCIFR]  "I" (_SFR_IO_ADDR(PCIFR) )
    );
};

/////////////////////////////////////////////////////////////////////////////////////////////////
int main (void) {
   setup();
   for (;;) loop ();
}

/*  ___      _
   / __| ___| |_ _  _ _ __
   \__ \/ -_)  _| || | '_ \
   |___/\___|\__|\_,_| .__/
                     |_|
*/
void setup(void) {
    clearMatrix();
    //Teensy++ 2.0 CPU initialization
    cli();
    CLKPR = 0x80;
    CLKPR = CPU_PRESCALER;
    // End of CPU initialization
    // Enable digital input on ADC ports
    DIDR0 = 0;
    // End of Teensy configuration
    //Make sure internal pull up funcionality is enabled:
    //MCUCR &= ~(_BV(PUD)); // not needed. this is the default
    // Interrupt pins (INT0..7)
    DDRD =  0xF0; // D0..D3 input, the rest output
    PORTD = 0x0F; // D0..D3 pullup
    DDRE =  0x0F; // E4..E7 input
    PORTE = 0xF0; // E4..E7 pullup
    //This one is Y8, but using a pin change interrupt.
    DDRB = 0xFE; // PB0 input. other output
    PORTB = _BV(PB0); // PB0 pullup
    DDRD |= _BV(PD6); //Port 6 OUTPUT
    // Output pins (simulates matrix)
    DDRC = 0xFF; // PORTC (10 to 18 in teensy++ 2.0) is the column output
    PORTC = 0xFF; // write high on all ports
    UCSR1B = 0; // disable USART
    //keyboard initialization
    if (!initalizeKeyboard()){
        #ifdef DEBUGMODE
        debug ("No keyboard!");
        #endif
        poweroff();
    }
    // Configure Interrupts
    EIMSK  = 0x0; // disable external interrupts when configuring
    EIFR = 0x0; // clear any pending interrupt
    EICRA = 0x55; //any edge on INT0..3
    EICRB = 0x55; //any edge on INT4..7
    //EICRA = 0xAA; //falling edge on INT0..3
    //EICRB = 0xAA; //falling edge on INT4..7
    EIMSK  = 0xFF; // enable all external interrupts
    PCICR  = (1 << PCIE0); // enable pin change interrupts
    PCIFR |= (1 << PCIF0); // clear any pending interrupt
    PCMSK0 = 0x01;        // PB0 (PCINT0) change interrupt for the keyboard
    TIMSK0 = 0;           // disable Timer 0 interrupts generated by Arduino
    sei();                // enable interrupts
}

/*  _                   ____
   | |   ___  ___ _ __ / /\ \
   | |__/ _ \/ _ \ '_ \ |  | |
   |____\___/\___/ .__/ |  | |
                 |_|   \_\/_/
*/
void loop(void) {
  uint8_t code, m;
  for (;;) { /* ever */
    /* Get a keycode from the keyboard and convert and update Keyboard Map */
    code = readPS2();
    #ifdef DEBUGMODE
    debug ("["); printHex (code); debug ("]");
    #endif
    // Clean keyboard matrix on buffer overflows
    if ( (code == 0x00) | (code == 0xff) ) {  // Error codes
      #ifdef DEBUGMODE
      debug ("!\n");
      #endif
      clearMatrix();
    } else { // not error codes
      if (code == 0xE0) {
        EXT = true; //Serial.println ("extend");
      } else if (code == 0xF0) {
        BRK = true; //Serial.println ("release");
      } else {
        if (EXT == true) { // extended keys
          EXT = false;
          switch (code) {
            case _PS2_UP_ARROW:     m = _UP      ; break;
            case _PS2_DOWN_ARROW:   m = _DOWN    ; break;
            case _PS2_LEFT_ARROW:   m = _LEFT    ; break;
            case _PS2_RIGHT_ARROW:  m = _RIGHT   ; break;
            case _PS2_RIGHT_ALT:    m = _CODE    ; break;
            case _PS2_RIGHT_CTRL:   m = _SELECT  ; break;
            case _PS2_LEFT_GUI:     m = _CAPS    ; break;
            case _PS2_RIGHT_GUI:    m = _GRAPH   ; break;
            case _PS2_KPSLASH:      m = _KPSLASH ; break;
            case _PS2_KPENTER:      m = _ENTER   ; break;
            case _PS2_END:          m = _STOP    ; break;
            case _PS2_HOME:         m = _HOME    ; break;
            case _PS2_INSERT:       m = _INSERT  ; break;
            case _PS2_DELETE:       m = _DELETE  ; break;
            default:                m = _NONE    ; break;
          } // switch
        } else { // normal set
          if (code < 128) {
            m = pgm_read_byte(PS2Keymap_Normal + code);
          } else {
              m = _NONE;
          }
        }  // end of normal set
        if (m != _NONE) {
            updateMatrix(m);
        }
        
        #ifdef DEBUGMODE
        debug ("<"); printHex (m); debug (">\n");
        #endif
      }
    } // end of "not error codes"
  }
}

//     __              _   _
//    / _|_  _ _ _  __| |_(_)___ _ _  ___
//   |  _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  \_,_|_||_\__|\__|_\___/_||_/__/
//
void poweroff() {
  EIMSK  = 0x0; // disnable all external interrupts
  PCMSK0 = 0x0;        // disable pin change interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();                         //stop interrupts to ensure the BOD timed sequence executes as required
  sleep_enable();
  sleep_cpu();
;}

inline void setColumn(uint8_t column, uint8_t event)
{
    switch(column){
        case 0:
            COLUMN_STATUS_0 = event;
            break;
        case 1:
            COLUMN_STATUS_1 = event;
            break;
        case 2:
            COLUMN_STATUS_2 = event;
            break;
        case 3:
            COLUMN_STATUS_3 = event;
            break;
        case 4:
            COLUMN_STATUS_4 = event;
            break;
        case 5:
            COLUMN_STATUS_5 = event;
            break;
        case 6:
            COLUMN_STATUS_6 = event;
            break;
        case 7:
            COLUMN_STATUS_7 = event;
            break;
        case 8:
            COLUMN_STATUS_8 = event;
            break;
        default:
            break;
        }
}

inline uint8_t* getColumn(uint8_t column)
{
    uint8_t* selectedColumn = &COLUMN_STATUS_0;
    switch(column){
        case 0:
            selectedColumn = &COLUMN_STATUS_0;
            break;
        case 1:
            selectedColumn = &COLUMN_STATUS_1;
            break;
        case 2:
            selectedColumn = &COLUMN_STATUS_2;
            break;
        case 3:
            selectedColumn = &COLUMN_STATUS_3;
            break;
        case 4:
            selectedColumn = &COLUMN_STATUS_4;
            break;
        case 5:
            selectedColumn = &COLUMN_STATUS_5;
            break;
        case 6:
            selectedColumn = &COLUMN_STATUS_6;
            break;
        case 7:
            selectedColumn = &COLUMN_STATUS_7;
            break;
        case 8:
            selectedColumn = &COLUMN_STATUS_8;
            break;
        default:
            break;
        }
    return selectedColumn;
}

//
// Test for connection and Initialize Keyboard
// Returns false on success
// and true on error
//
bool initalizeKeyboard( void )
{
  uint8_t ack;
  _delay_ms(1000); //It takes around 750ms to start
  // First... lets see if it's connected
  releaseCLK();
  releaseDAT();
  _delay_us(50);
    // http://www.burtonsys.com/ps2_chapweske.htm
  // 1)   Bring the Clock line low for at least 100 microseconds.
  dropCLK();
  _delay_us(150);
  // 2)   Bring the Data line low.
  dropDAT();
  _delay_us(10);
  // 3)   Release the Clock line.
  releaseCLK();
  _delay_us(5); // give some time for the line to raise
  // 4)   Wait for the device to bring the Clock line low.
  long timeout = 50000; // we loop to make up some time
  do {
      timeout--;
  } while ( (readCLK()) && (timeout));
  if (timeout) {
      dropDAT();
      _delay_ms(150); // lets cancel everything
      writePS2(0xff);  // send reset code
      ack = readPS2();  // byte, kbd does self test, returns ACK
      ack = readPS2();  // another ack when self test is done
      if (ack == 0xAA) { //0xAA= PS2_KC_BAT success code
          return true;
      }
  }
  return false;
}
// not really atomic, but works for us 
// 1 __tmp_reg__ holds the value
// 2 disable interrupts
// 3 check if column is clear (selected) 
// 4 jump forward if not clear
// 5 else set the value 
// 6 restore interrupts
#define atomicCheckAndSetPortC(port, column, value)  \
            asm volatile( \
            "ld __tmp_reg__,%a0" "\n\t"                      \
            "cli" "\n\t"                                   \
            "sbic %[_PIN], %[_COLUMN]" "\n\t"                  \
            "rjmp  1f" "\n\t"             \
            "out %[_PORTC],__tmp_reg__" "\n\t"           \
            "1:" "\n\t"   \
            "sei" "\n\t"                                 \
            ::[_NVALUE]   "e" ( value  ), \
            [_COLUMN]   "I" ( column  ), \
            [_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ), \
            [_PIN]   "I" (_SFR_IO_ADDR(port)  ) \
            )


// sets the port without being interrupted. Prevents setting the port
// after a raising edge.
// if a raising edge happens, it will clear the port AFTER we set it here
// which is what we want.
inline void setPortAtomic(uint8_t column, uint8_t * value) {
    
    switch (column) {
        case 0:
            atomicCheckAndSetPortC(PIND,0,value);
            break;
        case 1:
            atomicCheckAndSetPortC(PIND,1,value);
            break;
        case 2:
            atomicCheckAndSetPortC(PIND,2,value);
            break;
        case 3:
            atomicCheckAndSetPortC(PIND,3,value);
            break;
        case 4:
            atomicCheckAndSetPortC(PINE,4,value);
            break;
        case 5:
            atomicCheckAndSetPortC(PINE,5,value);
            break;
        case 6:
            atomicCheckAndSetPortC(PINE,6,value);
            break;
        case 7:
            atomicCheckAndSetPortC(PINE,7,value);
            break;
        case 8:
            atomicCheckAndSetPortC(PINB,0,value);
            break;
        default:
            break;
        }
    
}

//
// Activate the key addressed by m
//
void clearBit(uint8_t col, uint8_t lin) {
  #ifdef DEBUGMODE
  softSendByte('T');
  #endif
  uint8_t * column = getColumn(col);
  *column &= ~(1 << lin);
  setPortAtomic(col, column);
  
  #ifdef DEBUGMODE
  softSendByte('\\');
  #endif
}

//
// Deactivate the key addressed by m
//
void setBit(uint8_t col, uint8_t lin) {
  #ifdef DEBUGMODE
  softSendByte('_');
  #endif
  uint8_t * column = getColumn(col);
  *column |= (1 << lin);
  setPortAtomic(col, column);
  #ifdef DEBUGMODE
  softSendByte('/');
  #endif
}

//
// Clear the Keyboard Matrix
//
void clearMatrix( void )
{
  COLUMN_STATUS_0 = 255;
  COLUMN_STATUS_1 = 255;
  COLUMN_STATUS_2 = 255;
  COLUMN_STATUS_3 = 255;
  COLUMN_STATUS_4 = 255;
  COLUMN_STATUS_5 = 255;
  COLUMN_STATUS_6 = 255;
  COLUMN_STATUS_7 = 255;
  COLUMN_STATUS_8 = 255;
}

//
// Update the Keyboard Matrix based on a map code for a row and a line
//
void updateMatrix(uint8_t k) {
  uint8_t col = COLUMN_FROM_EVENT(k) ;
  uint8_t lin = ROW_FROM_EVENT(k);
  if (BRK == true) { // break code
    BRK = false;
    setBit(col, lin);
  } else {  // make code
    clearBit(col, lin);
  }
}

#ifdef DEBUGMODE
void initsoftSend(void) {
  DDRUART |= (1 << BITUART);
  PORTUART |= (1 << BITUART);
}

void debug( char *s) {
  char *ptr = s;
  while (*ptr) {
    if (*ptr == '\n') softSendByte ('\r');
    softSendByte( *ptr);
    ptr++;
  }
}

void softSendByte (uint8_t dado) {
  // 9600 bauds
  uint16_t shiftRegister;
  uint8_t i;
  // 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
  shiftRegister  = 0xff00 | (uint16_t)dado;  // 1  1  1  1  1  1  1  1 [         dado         ]
  shiftRegister <<= 1;                       // 1  1  1  1  1  1  1 [         dado         ] 0
  for (i = 0; i < 12; i++) { // 2 stop bits
    if (shiftRegister & 1)  PORTUART |= (1 << BITUART); else PORTUART &= ~(1 << BITUART);
    shiftRegister >>= 1;
    _delay_us (BIT_DELAY);
  }
}

void printHex ( uint8_t h) {
  char table[16] = {"0123456789ABCDEF"};
  uint8_t nibble = h >> 4;
  softSendByte (table [ nibble ] );
  softSendByte (table [ h & 0x0f]  );
}

void printBin ( uint8_t l) {
  uint8_t mask = 0x80;
  for (uint8_t j = 0; j < 8; j++) {
    if (l & mask) softSendByte ('1'); else softSendByte ('0');
    softSendByte (' ');
    mask >>= 1;
  }
}
#endif
