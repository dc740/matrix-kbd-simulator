/*
   PS2 polling/blocking library. no interrupts used.
   * Based on Arduino 'PS2' from Chris J. Kiick and 
   * a reimplementation from Daniel Jose Viana (danjovic@hotmail.com)
*/
#pragma once

// Hardware definition
#define PS2_DAT_DDR  DDRB
#define PS2_DAT_PIN  PINB
#define PS2_DAT_PORT PORTB
#define PS2_DAT_BIT  2

#define PS2_CLK_DDR  DDRB
#define PS2_CLK_PIN  PINB
#define PS2_CLK_PORT PORTB
#define PS2_CLK_BIT  1

#define dropDAT()    do { PS2_DAT_DDR  |=  (1 << PS2_DAT_BIT); PS2_DAT_PORT &= ~(1 << PS2_DAT_BIT); } while(0)
#define dropCLK()    do { PS2_CLK_DDR  |=  (1 << PS2_CLK_BIT); PS2_CLK_PORT &= ~(1 << PS2_CLK_BIT); } while(0)

#define releaseDAT() do {  PS2_DAT_DDR  &= ~(1 << PS2_DAT_BIT); PS2_DAT_PORT |=  (1 << PS2_DAT_BIT); } while (0)
#define releaseCLK() do {  PS2_CLK_DDR  &= ~(1 << PS2_CLK_BIT); PS2_CLK_PORT |=  (1 << PS2_CLK_BIT); } while (0)

#define readDAT()  (PS2_DAT_PIN & (1 << PS2_DAT_BIT))
#define readCLK()  (PS2_CLK_PIN & (1 << PS2_CLK_BIT))
//
#define waitDATrise()  do {} while (!readDAT())
#define waitDATfall()  do {} while ( readDAT())

#define waitCLKrise()  do {} while (!readCLK())
#define waitCLKfall()  do {} while ( readCLK())

// Pretty much the same as writePS2, except with a timeout
// and verification.
// Checks that the keyboard is connected and correctly initialized
// returns true on success. false otherwise
bool initPS2();

// Send one byte to the device. Blocking implementation.
void writePS2(uint8_t data); 

// Receive one byte from the device. Blocking implementation.
uint8_t readPS2(void);           
