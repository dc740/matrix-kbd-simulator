//select only one of these
#define USE_C0_WORKAROUND_1 //set c0 2ms before the interrupt
//#define USE_C0_WORKAROUND_2 //set c0 98us after Y8 goes down (glitch or something?)

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

/* first output on matrix. Interrupt on falling edge goes here
 *  NOTE: there are Y9 and Y10, but the Sony HB10p doesn´t connect them to the matrix
 */
const int MAT_INT_PIN =  0; //Pin that fires the interrupt. Y0 normally, or Y8 here if using workaround 3
const int TOTAL_Y_PINS = 9; // Y0..Y8
const int OUTPUT_LOW_DURATION_MICROS = 18; // each column low time. I think it should be 17.66666

//X0..X7 pins are the inputs in the MSX, so they will be our OUTPUTS here
const int TOTAL_X_PINS = 8;
//each bit of a column is a row status. 0 means key pressed
byte COLUMN_STATUS[9] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const byte COLUMN_DELAYS[9] = {
#ifdef USE_C0_WORKAROUND_2
  OUTPUT_LOW_DURATION_MICROS, //the interrupt happens way before on this one
#else
OUTPUT_LOW_DURATION_MICROS - 4, // there is a delay to turn on the first column of 3.5us so we leave it on for less
#endif
  OUTPUT_LOW_DURATION_MICROS,
  OUTPUT_LOW_DURATION_MICROS-1, // -1 to correct deviation on latest rows
  OUTPUT_LOW_DURATION_MICROS,
  OUTPUT_LOW_DURATION_MICROS-1,
  OUTPUT_LOW_DURATION_MICROS,
  OUTPUT_LOW_DURATION_MICROS-1,
  OUTPUT_LOW_DURATION_MICROS-1,
  OUTPUT_LOW_DURATION_MICROS};

#ifdef USE_C0_WORKAROUND_1
// This is a workaround to set the first column status a few milliseconds BEFORE the interrupt triggers
unsigned long time_since_interrupt;
const unsigned long PRE_SET_COLUMN_0_TIME = 18; // we know the interrupt occurs every 20ms, so we set column 0 after 18 since the last run.
#endif


// DEMO VARIABLES
// lets press and release each button at least once
unsigned long time_since_event;
byte demo_events[TOTAL_Y_PINS];
byte current_column; //Y0..Y8
byte current_row; //(X0..X7)
const int ledPin =  LED_BUILTIN;
int ledState = LOW;
bool buttonPressed = false;
unsigned long current_time;
unsigned long delay_time;
byte event;

void reading_process_started(){
  // loop unroll optimization...
#ifdef USE_C0_WORKAROUND_2
// connect Y8 to the interrupt pin. thos pinm goes low for 98us
//before the keyboard matrix is scanned
  delayMicroseconds(94); // set the pin around 98us after the interrupt (94+4)
#endif
  
#ifndef USE_C0_WORKAROUND_1
// for column 0, we set this about 2ms before in a workaround so it reads fine and doesn´t generate weird chars
// but if the workaround is disabled, you need to set it here
  PORTC = COLUMN_STATUS[0];
#endif
  delayMicroseconds(COLUMN_DELAYS[0]);
  PORTC = COLUMN_STATUS[1];
  delayMicroseconds(COLUMN_DELAYS[1]);
  PORTC = COLUMN_STATUS[2];
  delayMicroseconds(COLUMN_DELAYS[2]);
  PORTC = COLUMN_STATUS[3];
  delayMicroseconds(COLUMN_DELAYS[3]);
  PORTC = COLUMN_STATUS[4];
  delayMicroseconds(COLUMN_DELAYS[4]);
  PORTC = COLUMN_STATUS[5];
  delayMicroseconds(COLUMN_DELAYS[5]);
  PORTC = COLUMN_STATUS[6];
  delayMicroseconds(COLUMN_DELAYS[6]);
  PORTC = COLUMN_STATUS[7];
  delayMicroseconds(COLUMN_DELAYS[7]);
  PORTC = COLUMN_STATUS[8];
  delayMicroseconds(COLUMN_DELAYS[8]);
  
  //PORTC = COLUMN_STATUS[0]; // leave column 0 pressed, so it can read it when the interrupt starts
  PORTC = 0xff; // prevents weird chars?

#ifdef USE_C0_WORKAROUND_1
  //this workaround allows us to set the COLUMN 0 a few milliseconds before the interrupt fires again
  time_since_interrupt = millis();
#endif
}

void setup() {
    //setup inputs (we only need one MAT_INT_PIN. the rest are for the logic analyzer
    //pinMode(MAT_INT_PIN, INPUT_PULLUP);
    //this port is where the interrupt comes    
    DDRD = 0x0; // PORTD (0 to 7 in teensy++ 2.0) is input
    PORTD = 0xFF; // pullup
    pinMode(8, INPUT_PULLUP); //E0

    // Lets use the entire PORTC in the teensy
    // These pins trick the MSX so it thinks it's the matrix
    DDRC = 0xFF; // PORTC (10 to 18 in teensy++ 2.0) is output
    PORTC = 0xFF; // write high on all ports

    delay(10000);
    // finally, enable the interrupt and start simulating the keyboard
    attachInterrupt(digitalPinToInterrupt(MAT_INT_PIN), reading_process_started, FALLING);
    //DEMO CODE
    // create events for all keys, column by column, row by row
    time_since_event = millis();
    
    current_column = 0;
    current_row = 0;
    
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, ledState);

}

// this is mostly a demo
void loop() {
  current_time = millis();

  if (buttonPressed) {
    delay_time = 22;
  } else {
    delay_time = 1000;
  }
  if (current_time - time_since_event > delay_time) {
    // do new event
    event = 0xFF;
    if (!buttonPressed) { //we have to press the button
      bitClear(event, current_row);
      // press the button but stay on the same row and column, so we can release later
      buttonPressed = true;
    } else { //we are releasing
      // Set everything high and move to the next row
      current_row++;
      buttonPressed = false;
    }
    COLUMN_STATUS[current_column] = event;
    time_since_event = current_time;
    
    
    if (current_row >= TOTAL_X_PINS) {
      current_row = 0;
      current_column++;
      if (current_column >= TOTAL_Y_PINS) {
        current_column = 0;
        delay(2000); // wait and switch led, so we know we start over
        // led for demo effect
        if (ledState == LOW) {
          ledState = HIGH;
        } else {
          ledState = LOW;
        }
    
        // set the LED with the ledState of the variable:
        digitalWrite(ledPin, ledState);
      }
    }
    
  }

#ifdef USE_C0_WORKAROUND_1
  //noInterrupts();
  if (current_time - time_since_interrupt > PRE_SET_COLUMN_0_TIME) {
    //we know the interrupt will fire in about 2ms, so lets set PORTC
    PORTC = COLUMN_STATUS[0];
    time_since_interrupt = current_time; // prevent refiring this again until next real interrupt
  }
  //interrupts();
#endif
  

}
