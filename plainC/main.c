/***
         _______  ______  ____    ______
        | ____\ \/ /  _ \/ ___|  / /___ \
        |  _|  \  /| |_) \___ \ / /  __) |
        | |___ /  \|  __/ ___) / /  / __/
        |_____/_/\_\_|   |____/_/  |_____|

       Teclado PS/2 externo para Expert XP800
       Danjovic 2021 
       05 april 2021 - basic release
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define BIT_DLY_9600   103
#define BIT_DLY_19200  51
#define BIT_DLY_38400  25
#define BIT_DLY_57600  17
#define BIT_DLY_115200 8
#define BIT_DELAY      BIT_DLY_9600

#define DDRUART        DDRB
#define PORTUART       PORTB
#define BITUART        4


#include "msxKeyboardMatrix.h"
#include "ps2keyMap.h"
#include "myPS2.h"             // pin definition on the file
//#include <ps2.h>
//PS2 kbd(9, 8);

#define NOP() do { __asm__ __volatile__ ("nop"); } while (0)

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
#error "Teensyduino only supports 16, 8, 4, 2, 1 MHz.  Check your settings"
#endif

// DEMO VARIABLES
uint8_t current_column=0; //Y0..Y8
uint8_t current_row=0; //(X0..X7)
uint8_t event=0xFF;
// END OF DEMO VARIABLES

//       _      __ _      _ _   _
//    __| |___ / _(_)_ _ (_) |_(_)___ _ _  ___
//   / _` / -_)  _| | ' \| |  _| / _ \ ' \(_-<
//   \__,_\___|_| |_|_||_|_|\__|_\___/_||_/__/
//
#define Keymap_Size 16  // 16 up to lines of keys for Hotbit 
 

void printBin ( uint8_t l);
void printHex ( uint8_t l); 
void printMatrix(void );
void softSendByte (uint8_t dado);
void debug( char *s);
void initsoftSend(void);
void updateMatrix(uint8_t k);
void clearMatrix( void );
void initalizeKeyboard(void);
void setup(void);
void loop(void);

//                 _      _    _
//   __ ____ _ _ _(_)__ _| |__| |___ ___
//   \ V / _` | '_| / _` | '_ \ / -_|_-<
//    \_/\__,_|_| |_\__,_|_.__/_\___/__/
//

static volatile uint8_t cc = 5;
//test to toggle just one bit
volatile uint8_t COLUMN_STATUS_0 = 255; 
volatile uint8_t COLUMN_STATUS_1 = 255;
volatile uint8_t COLUMN_STATUS_2 = 255; 
volatile uint8_t COLUMN_STATUS_3 = 255;
volatile uint8_t COLUMN_STATUS_4 = 255; 
volatile uint8_t COLUMN_STATUS_5 = 255;
volatile uint8_t COLUMN_STATUS_6 = 255; 
volatile uint8_t COLUMN_STATUS_7 = 255;
volatile uint8_t COLUMN_STATUS_8 = 255;
bool EXT = false;
bool BRK = false;
bool SHIFT = false;
bool LAST_SHIFT = false;



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
    
    "ldi r27,hi8(COLUMN_STATUS_0)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_0)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_1)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_1)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_2)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_2)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_3)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_3)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_4)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_4)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_5)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_5)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_6)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_6)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    
    "ldi r27,hi8(COLUMN_STATUS_7)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_7)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_EIFR],1 \n\t"           // reset interrupt bit.

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
   "rjmp .raising_edge\n\t"
   //now we go back to the usual ISR that a normal interrupt uses
   // Falling edge
    ".falling_edge:\n\t"
    "ldi r27,hi8(COLUMN_STATUS_8)\n\t"
    "ldi r26,lo8(COLUMN_STATUS_8)\n\t"
    "ld 26,X \n\t"
    "out %[_PORTC] ,r26 \n\t"
    "rjmp .exit_isr\n\t"
    ".raising_edge:\n\t"
    // now we set the signal back to 0xFF IF no other column is ON (LOW really)
    
    "in	r26,%[_PIND]\n\t" //all Y0..Y3 are located on PORTD
    "in	r27,%[_PINE]\n\t" //all Y4..Y7 are located on PORTE
    "andi r26,0x0F\n\t" //keep Y0..Y3
    "andi r27,0xF0\n\t" //keep Y4..Y7
    "or r26,r27\n\t"
    "cpi r26,0xFF\n\t" //if all is HIGH
    "brne .exit_isr\n\t" //if Y4...Y7 is NOT all HIGH, we exit
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
    ".exit_isr:"
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

void setupTimer1() {
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 56939.50177935943 Hz (16000000/((280+1)*1))
  OCR1A = 280;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 1
  TCCR1B |= (1 << CS10);
  // Output Compare Match A Interrupt Enable
  
  //Enable this from the Y8 interrupt
  // and disable it in the timer interrupt handler
  //TIMSK1 |= (1 << OCIE1A);
}

void setup(void) {
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
    PORTD |= _BV(PD6); //Port 6 HIGH

    // Output pins (simulates matrix)
    DDRC = 0xFF; // PORTC (10 to 18 in teensy++ 2.0) is the column output
    PORTC = 0xFF; // write high on all ports
    
    UCSR1B = 0; // disable USART

    // Configure Interrupts
    EIMSK  = 0x0; // disable external interrupts when configuring
    EIFR = 0x0; // clear any pending interrupt
    EICRA = 0xAA; //falling edge on INT0..3
    EICRB = 0xAA; //falling edge on INT4..7
    EIMSK  = 0xFF; // enable all external interrupts



    PCICR  = (1 << PCIE0); // enable pin change interrupts
    PCIFR |= (1 << PCIF0); // clear any pending interrupt
    PCMSK0 = 0x01;        // PB0 (PCINT0) change interrupt for the keyboard

    TIMSK0 = 0;           // disable Timer 0 interrupts generated by Arduino
    sei();                // enable interrupts
    
    _delay_ms(10000);
}






/*  _                   ____
   | |   ___  ___ _ __ / /\ \ 
   | |__/ _ \/ _ \ '_ \ |  | |
   |____\___/\___/ .__/ |  | |
                 |_|   \_\/_/
*/

inline void set_column(uint8_t column, uint8_t event)
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




void loop(void) {
    // led for demo effect
    PORTD ^= _BV(PD6); //Port D6 (led) toggle
    
    //DEMO CODE
    // do new event
    event = 0xFF;
    //bitClear
    event &= ~(1 << current_row);
    // press button (only the status. the interrupt replicates the status in the port)
    set_column(current_column, event);
    _delay_ms(40);
    // release button
    set_column(current_column, 0xFF);

    // test next char
    current_row++;
    if (current_row >= 8) {
      current_row = 0;
      current_column++;
      if (current_column >= 9) {
        current_column = 0;
        _delay_ms(10000); // wait a lot, so we know we start over
        
      }
    }
    _delay_ms(1000);
}

