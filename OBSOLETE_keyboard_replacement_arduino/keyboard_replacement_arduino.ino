#include <PS2KeyAdvanced.h>
#include "PS2ToMSXTranslator.h"
#include <CircularBuffer.h>
#include <set.h>
#include <avr/sleep.h>

//select only one of these
#define USE_C0_WORKAROUND_1 //set c0 2ms before the interrupt
//#define USE_C0_WORKAROUND_2 //set c0 98us after Y8 goes down (glitch or something?)

#define DISPLAY_REFRESH_RATE 50

//Teensy++ 2.0
#define KEYBOARD_DATA_PIN 20
#define KEYBOARD_INT_PIN 19


/*
 * WORKAROUNDS DESCRIPTION
 * 1) set the fist column status 2ms before the interrupt fires
 * Works OK. There is a 100us undocumented signal on Y8 right before Y0 starts.
 * I found no information about it. Seems to work OK with the status being set for Y0 at the same time.
 * So far, it's the best workaround
 * 3) Uses the Y8 signal mentioned above to wait for 98us and set the column 0 status.
 * I haven't tested it, but I don't like that the interrupt takes 260us to run instead of 160us.
 * The PS2 keyboard also works with interrupts, so I don't want to spend the extra time.
 * Also, Y8 gets called too, so this should not be used, or the handler will be triggered again.
 * TODO: fix this workaround to not trigger right after it finished
 */

// Pin that fires the interrupt. Y0 normally, or Y8 here if using workaround 3
#define MAT_INT_PIN  0 
// Y0..Y8
#define TOTAL_Y_PINS 9
// X0..X7 pins are the inputs in the MSX, so they will be our OUTPUTS here
#define TOTAL_X_PINS 8
// each column low time. I measured 17.66666
#define OUTPUT_LOW_DURATION_MICROS 18


#ifdef USE_C0_WORKAROUND_2
//the interrupt happens way before with this workaround, no need to take 4us
  #define COLUMN_DELAY_0 OUTPUT_LOW_DURATION_MICROS 
#else
// there is a delay to turn on the first column of 3.5us so we leave it on for less
  #define COLUMN_DELAY_0 OUTPUT_LOW_DURATION_MICROS - 4
#endif
#define COLUMN_DELAY_1 OUTPUT_LOW_DURATION_MICROS
// -1 to correct deviation on latest rows
#define COLUMN_DELAY_2 OUTPUT_LOW_DURATION_MICROS-1
#define COLUMN_DELAY_3 OUTPUT_LOW_DURATION_MICROS
#define COLUMN_DELAY_4 OUTPUT_LOW_DURATION_MICROS-1
#define COLUMN_DELAY_5 OUTPUT_LOW_DURATION_MICROS
#define COLUMN_DELAY_6 OUTPUT_LOW_DURATION_MICROS-1
#define COLUMN_DELAY_7 OUTPUT_LOW_DURATION_MICROS-1
#define COLUMN_DELAY_8 OUTPUT_LOW_DURATION_MICROS

#define COLUMN_ROW_INDEX(c,r) (c+TOTAL_Y_PINS*r)


//each bit of a column is a row status. 0 means key pressed
byte COLUMN_STATUS[9] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
unsigned long last_interrupt_time;
unsigned long last_update_time;
unsigned long current_time;
CircularBuffer<uint16_t, 100> event_buffer; //a totally overkill solution to a non existent problem I want to solve.
Set current_keys_in_status; //another example of over engineering. We keep track which keys we are sending at each time


#ifdef USE_C0_WORKAROUND_1
// This is a workaround to set the first column status a few milliseconds BEFORE the interrupt triggers
unsigned long w1_last_interrupt_time; //to keep track when we last ran the workaround
const unsigned long PRE_SET_COLUMN_0_TIME = 1000/DISPLAY_REFRESH_RATE - 3; //3ms before the interrupt
#endif

//PS2 keyboard related variables
PS2KeyAdvanced keyboard;
uint16_t scanCode;
uint16_t current_event;
uint8_t lastPressedKey; // manually disable typematic. Lets ignore repeated presses (without release) of the last key

/**
* this is as close as we can get to a shutdown, so we don't use energy when the keyboard is not connected.
* This function does not work for some boards. Works fine on Teensy++ 2.0
*/
void poweroff() {
  detachInterrupt(digitalPinToInterrupt(KEYBOARD_INT_PIN));
  detachInterrupt(digitalPinToInterrupt(MAT_INT_PIN));
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();                         //stop interrupts to ensure the BOD timed sequence executes as required
  sleep_enable();
  sleep_cpu();
  digitalWrite(LED_BUILTIN, HIGH);
;}

void reading_process_started(){
  // loop unroll optimization...
#ifdef USE_C0_WORKAROUND_2
// connect Y8 to the interrupt pin. thos pinm goes low for 98us
//before the keyboard matrix is scanned
//TODO: fix so it doesn´t fire again until next refresh
  delayMicroseconds(94); // set the pin around 98us after the interrupt (94+4)
#endif
  
#ifndef USE_C0_WORKAROUND_1
// for column 0, we set this about 2ms before in a workaround so it reads fine and doesn´t generate weird chars
// but if the workaround is disabled, you need to set it here
  PORTC = COLUMN_STATUS[0];
#endif
  delayMicroseconds(COLUMN_DELAY_0);
  PORTC = COLUMN_STATUS[1];
  delayMicroseconds(COLUMN_DELAY_1);
  PORTC = COLUMN_STATUS[2];
  delayMicroseconds(COLUMN_DELAY_2);
  PORTC = COLUMN_STATUS[3];
  delayMicroseconds(COLUMN_DELAY_3);
  PORTC = COLUMN_STATUS[4];
  delayMicroseconds(COLUMN_DELAY_4);
  PORTC = COLUMN_STATUS[5];
  delayMicroseconds(COLUMN_DELAY_5);
  PORTC = COLUMN_STATUS[6];
  delayMicroseconds(COLUMN_DELAY_6);
  PORTC = COLUMN_STATUS[7];
  delayMicroseconds(COLUMN_DELAY_7);
  PORTC = COLUMN_STATUS[8];
  delayMicroseconds(COLUMN_DELAY_8);

  PORTC = 0xff; // release everything. To truly simulate the matrix
  last_interrupt_time = millis();
#ifdef USE_C0_WORKAROUND_1
  //this workaround allows us to set the COLUMN 0 a few milliseconds before the interrupt fires again
  w1_last_interrupt_time = last_interrupt_time;
#endif
}

void setup() {
    //Serial.begin(9600);
    //setup inputs (we only need one MAT_INT_PIN. the rest are for the logic analyzer
    //pinMode(MAT_INT_PIN, INPUT_PULLUP);
    //this port is where the interrupt comes    
    //Declare all this here because Iḿ using these to debug. Only MAT_INT_PIN is needed
    pinMode(0, INPUT_PULLUP); //D0  <<< usually this is MAT_INT_PIN
    pinMode(1, INPUT_PULLUP); //D1
    pinMode(2, INPUT_PULLUP); //D2
    pinMode(3, INPUT_PULLUP); //D3
    pinMode(4, INPUT_PULLUP); //D4
    pinMode(5, INPUT_PULLUP); //D5
    pinMode(6, OUTPUT);       //D6 is the LED_BUILTIN in the teensy++ 2.0
    digitalWrite(6, LOW);
    pinMode(7, INPUT_PULLUP); //D7 
    pinMode(8, INPUT_PULLUP); //E0
    pinMode(9, INPUT_PULLUP); //E1

    // Lets use the entire PORTC in the teensy
    // These pins trick the MSX so it thinks it's the matrix
    DDRC = 0xFF; // PORTC (10 to 18 in teensy++ 2.0) is output
    PORTC = 0xFF; // write high on all ports

    // finally, enable the interrupt and start simulating the keyboard
    attachInterrupt(digitalPinToInterrupt(MAT_INT_PIN), reading_process_started, FALLING);

    // now initialize the keyboard library
    keyboard.begin( KEYBOARD_DATA_PIN, KEYBOARD_INT_PIN );
    keyboard.echo( );              // ping keyboard to see if there
    delay( 1000 ); // I think 750 is more than enough, so... I wait a second (the MSX is booting anyway)
    scanCode = keyboard.read( );
    if( (scanCode & 0xFF) == PS2_KEY_ECHO 
        || (scanCode & 0xFF) == PS2_KEY_BAT ) {
        //Serial.println( "Keyboard OK.." );    // Response was Echo or power up
        }
    else {
      if( ( scanCode & 0xFF ) == 0 ){
        //Serial.println( "Keyboard Not Found" );
      }
      else
        {
        //Serial.print( "Invalid Code received of " );
        //Serial.println( scanCode, HEX );
        }
        poweroff(); // in release mode we would do this
    }
}

void loop() {
  current_time = millis();

  // read scancode from PS2 port and store it in the buffer
  if (current_time - last_interrupt_time  < 15) { // try avoiding calls to keyboard when we are going to be interrupted. Lets see if this fixes anything.
    while(( scanCode = keyboard.read( ) ))
    {
      if (!event_buffer.isFull()) {
        event_buffer.push(scanCode);
      } else {
        //we are losing events. Really? how fast are you typing? this is a world record.
      }
    }
  }

  // check if the MSX has read the previous status already. Otherwise skip
  if (last_interrupt_time > last_update_time) {
    // there was an update so we can start over processing keys
    current_keys_in_status.clear();
    //take the first item from the FIFO
    while (!event_buffer.isEmpty()) {
      current_event = event_buffer.first();
      uint8_t msx_key = translateEventToMatrix(current_event);
      if (msx_key == 0xFF) {
          //invalid key. There is no representation for the MSX
          event_buffer.shift(); 
          break; 
      }
      /*
      anti-typematic code. If typematic fills the event_buffer
      it could potentially slow down the queue. Better drop
      the repeated events to make sure
      */
      uint8_t character = current_event & 0xFF;
      if (~current_event & 0x8000) {
        // this a press
        if (lastPressedKey == character) {
          // this is a repeated press. The keyboard is in typematic mode
          event_buffer.shift(); //lets remove the repeated event
          //are we dropping the original event? no, it happened long ago, many interrupts ago
          break; //and skip any more calculations. They are not needed
        }
      }
      
      // the event already has the column and row in the last and first 8 bits
      byte column = COLUMN_FROM_EVENT(msx_key);
      byte row = ROW_FROM_EVENT(msx_key);
      if (!current_keys_in_status.has(msx_key)){
        event_buffer.shift();
        current_keys_in_status.add(msx_key);
        // set the status.. 
        if (current_event & 0x8000) { //check status of the last translation
          bitSet(COLUMN_STATUS[column], row);
          lastPressedKey = 0x00; // clear the typematic mechanism
        } else {
          bitClear(COLUMN_STATUS[column], row);
          lastPressedKey = character;
        }

      } else {
        // this key already has pending events to be processed. Finish this now to respect the event ordering.
        break;
      }
    }
    last_update_time = current_time;
  }
#ifdef USE_C0_WORKAROUND_1
  //noInterrupts();
  if (current_time - w1_last_interrupt_time > PRE_SET_COLUMN_0_TIME) {
    //we know the interrupt will fire in about 3ms, so lets set PORTC with the latest updates
    PORTC = COLUMN_STATUS[0];
    w1_last_interrupt_time = current_time; // prevent refiring this again until next real interrupt
  }
  //interrupts();
#endif
}
