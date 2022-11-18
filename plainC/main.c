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
uint8_t current_column; //Y0..Y8
uint8_t current_row; //(X0..X7)
uint8_t event;
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
    "out %[_PORTC] ,r26 \n\t"         // 1 escreve na porta B da PPI
                                     
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
    
   "ldi r27,hi8(COLUMN_STATUS_8)\n\t"        
    "ldi r26,lo8(COLUMN_STATUS_8)\n\t"        
    "ld 26,X \n\t"                   
    "out %[_PORTC] ,r26 \n\t"         // 1 escreve na porta B da PPI
                                     
    "in r26, %[_GPIOR1] \n\t"
    "in r27, %[_GPIOR2] \n\t"

    "sbi %[_PCIFR],1 \n\t"           // reset interrupt bit

    "out __SREG__,__zero_reg__ \n\t" 
    "lds __zero_reg__, zero \n\t"
    "reti \n\t"

    ::[_PORTC]   "I" (_SFR_IO_ADDR(PORTC)  ),
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
    switch(current_column){
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

