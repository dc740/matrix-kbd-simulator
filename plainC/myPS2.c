/*
   Biblioteca para dispositivos com protocolo PS/2
   Baseada na biblioteca Arduino 'PS2' da autoria de Chris J. Kiick
   Daniel Jose Viana - Abril 2021 - danjovic@hotmail.com
*/
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "myPS2.h"

//
// Pretty much the same as writePS2, except with a timeout
// and verification.
// Checks that the keyboard is connected and correctly initialized
// returns true on success. false otherwise
bool initPS2() {
  uint8_t ack;
  uint8_t data=0xff; //reset code
  uint8_t i;
  uint8_t parity = 1;

  // prepare for transmit
  releaseCLK();
  releaseDAT();
  _delay_us(200);

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
  long timeout = 100000; // we loop to make up some time
  do {
      timeout--;
  } while ( (readCLK()) && (timeout)); // same as waitCLKfall()
  if (timeout==0) {
      return false; // Keyboard not present
  }

  for (i = 0; i < 8; i++)
  {
    // 5)   Set/reset the Data line to send the first data bit
    if (data & 0x01)
    {
      releaseDAT();
      parity++;
    } else {
      dropDAT();
    }
    // 6)   Wait for the device to bring Clock high.
    waitCLKrise();

    // 7)   Wait for the device to bring Clock low.
    waitCLKfall();

    data >>= 1;

    // 8)   Repeat steps 5-7 for the other seven data bits
  } // for
  //
  // and the parity bit
  if (parity & 0x01)
  {
    releaseDAT();
  } else {
    dropDAT();
  }

  waitCLKrise();
  waitCLKfall();

  // 9)   Release the Data line.
  releaseDAT();
  _delay_us(10);

  // 10) Wait for the device to bring Data low.
  waitDATfall();

  // 11) Wait for the device to bring Clock  low.
  waitCLKrise();

  // 12) Wait for the device to release Data and Clock
  waitDATrise();
  waitCLKrise();

  // Hold data line low to hold keyboard until next action
  dropDAT();
  
  ack = readPS2();  // byte, kbd does self test, returns ACK
  ack = readPS2();  // another ack when self test is done
  if (ack == 0xAA) { //0xAA= PS2_KC_BAT success code
      return true;
  } else {
      return false;
  }
}


//
// Send a byte to the PS/2 device

void writePS2(uint8_t data) {
  uint8_t i;
  uint8_t parity = 1;

  // prepare for transmit
  releaseCLK();
  releaseDAT();
  _delay_us(200);

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
  waitCLKfall();



  for (i = 0; i < 8; i++)
  {
    // 5)   Set/reset the Data line to send the first data bit
    if (data & 0x01)
    {
      releaseDAT();
      parity++;
    } else {
      dropDAT();
    }
    // 6)   Wait for the device to bring Clock high.
    waitCLKrise();

    // 7)   Wait for the device to bring Clock low.
    waitCLKfall();

    data >>= 1;

    // 8)   Repeat steps 5-7 for the other seven data bits
  } // for
  //
  // and the parity bit
  if (parity & 0x01)
  {
    releaseDAT();
  } else {
    dropDAT();
  }

  waitCLKrise();
  waitCLKfall();

  // 9)   Release the Data line.
  releaseDAT();
  _delay_us(10);

  // 10) Wait for the device to bring Data low.
  waitDATfall();

  // 11) Wait for the device to bring Clock  low.
  waitCLKrise();

  // 12) Wait for the device to release Data and Clock
  waitDATrise();
  waitCLKrise();

  // Hold data line low to hold keyboard until next action
  dropDAT();
}


//
// Receive one byte from PS/2 device

uint8_t readPS2() {
  uint8_t data;
  uint8_t i;
  uint16_t shiftRegister = 0;

  uint16_t mask = 0x0001;

  // Prepare for receive
  releaseCLK();
  releaseDAT();
  _delay_us(50);

  // receive 11 bits
  for (i = 0; i < 11; i++)   {
    waitCLKfall();
    _delay_us(5);

    if ( readDAT() )
      shiftRegister |= mask;

    waitCLKrise();
    mask <<= 1;     // point next bit
  }

  // hold clock line to inhibt further data
  dropCLK();

  // todo test for parity, framing, etc
  data = (uint8_t) (shiftRegister >> 1);

  return data;
}
