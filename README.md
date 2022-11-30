# matrix-kbd-simulator

Matrix keyboard simulator for MSX Sony HB10 and HB20 using an Teensy++2.0 (@16Mhz).

I got tired of fixing the keyboard membrane, so I started to replace it with a PS2 keyboard.
The hardest part was getting the timing right.


# Setup:

## MSX to Teensy pinout
You need to connect X0-X7 and Y0-Y8 from the keyboard header in your MSX to your Teensy++ 2.0, and this app will simulate to be the keyboard.
Y0-Y7 -> D0..D3 and E4..E7, because we use int0...int7
Y8 goes to B0, using PCINT0.
Make sure you fix the pinout if you use different hardware. And also verify the ISRs and make sure you know when you are using pin
change interrupts, because if y=ou use them you have less ISR handlers to work with, and will need to modify them to find out
which pin triggered the interrupt.

| Header MSX | Pin | Port mode | Teensy++ 2.0 (AT90USB1286) port |
|:-----:|:------:|:------:|:------:|
| 8 |    Y0   |   Input pullup + Interrupt   | D0 |
| 9 |    Y1   |   Input pullup + Interrupt   | D1 |
| 19 |    Y2   |   Input pullup + Interrupt   | D2 |
| 14 |    Y3   |   Input pullup + Interrupt   | D3 |
| 15 |    Y4   |   Input pullup + Interrupt   | E4 |
| 13 |    Y5   |   Input pullup + Interrupt   | E5 |
| 21 |    Y6   |   Input pullup + Interrupt   | E6 |
| 6 |    Y7   |   Input pullup + Interrupt   | E7 |
| 5 |    Y8   |   Input pullup + Pin Change Interrupt   | B0 |
| 20 |    X0   |   Output   | C0 |
| 11 |    X1   |   Output   | C1 | 
| 18 |    X2   |   Output   |  C2 |
| 7 |    X3   |   Output   |  C3 |
| 16 |    X4   |   Output   |  C4 |
| 12 |    X5   |   Output   |  C5 |
| 10 |    X6   |   Output   |  C6 |
| 17 |    X7   |   Output   |  C7 |


## PS2 port pinout

| PS2 Pin | Teensy++ 2.0 (AT90USB1286) port |
|:------:|:------:|
| DATA | B2 |
| CLOCK | B1 |

5V and GND pins:
Please take GND from the properly labeled pins in the MSX and NOT the header!
There is no GND on the keyboard header. It looks like it, but it's not.
5V can be taken from the header without problems.

# Features:

* Simulates all key presses
* Each trigger works in around 1us.
* It powers down itself if the keyboard is missing to reduce power usage.
* Instantly updates the MSX if a currently selected column is updated

# How it works:

I got the idea while trying to fix a broken keyboard membrane, and confirmed it should work when I found someone else who already
did that too:
https://caro-su.translate.goog/msx/kbd4msx.htm?_x_tr_sch=http&_x_tr_sl=auto&_x_tr_tl=en&_x_tr_hl=en&_x_tr_pto=wapp

When Y0..Y8 from the keyboard header go low, each fire a falling edge interrupt in our AVR. Then the interrupt sets the bits on PORTC to
tell the MSX which keys are pressed or not.
The hardest part is to get the timing right. The Arduino takes 3.5us after the interrupt is fired until it sets the port with
our pressed keys, so I had to experiment with a few workarounds to overcome this problem, since it would skip the entire first
row if I didn't fix the 3.5us delay. In the end I reimplemented it in C heavily based on another project (listed below).

In order for the instant update to work, you need to take care of the timings on raising edge, and update the MSX
with the currently selected column (if any). I wrote a macro to prevent collisions and ensure we never set the column
after the raising edge interrupt fires, since this would mean that the PORTC would have the column value when it 
actually should be high, to prevent wrong keypresses during Y9 and Y10 (when the BIOS functionality is used).

The code was heavily based on the project below, but I eventually rewrote most of it to fix the speed and compatibility.
The Sony HB 10p and 20p also don't have any external connectivity for the keyboard and don't use a decoder like other models.
They select the matrix column directly from the S3527 to the keyboard membrane. I'm not sure what the other models use, but every
 single implementation I saw used a decoder to select the column on the membrane or had an external port that used a decoder too.
I originally implemented all the same in Arduino, but the performance was so bad.
that I started looking at other options, and this project did (almost) everything I wanted to do, so I rewrote the interrupt
handlers, parts of the logic, and reused the PS2 library:
https://hackaday.io/project/178358-exps2
https://github.com/Danjovic/MSX/tree/master/EXPS2/firmware/EXPS-2

# Layout

Adding an spanish layout would essentially mean to lose most of the special characters on the original keyboard.
So I created a layout that matches the original one, but on an spanish keyboard.

If anyone wants to add a new layout, please feel free to send a Pull Request.
Things to consider before starting such a project:
Each key on the spanish keyboard does NOT have a 1-1 match on the MSX and viceversa.
You will need to map a scan code from the PS2 keyboard (or USB if you have a host enabled device) and map it to
several MSX keys.
Per last statement, each keypress on the PS2 keyboard can result in SHIFT, GRAPH, CODE keys + another character, all in one single press,
but most important, the shift and Alt GR status from the PS2 keyboard must be completely independent from the same keys on the MSX keyboard.
Some examples:
'{' on the spanish keyboard is produced by pressing 'Alt Gr' + '´' key. And needs to be translated to 'SHIFT' + '\[' on the MSX matrix.
'`' on the spanish keyboard is produced by pressing '`' key. And needs to be translated to 'CODE' + '´' on the MSX matrix.
'^' on the spanish keyboard is produced by pressing 'SHIFT'+'`' key. And needs to be translated to 'CODE' + 'SHIFT' + '´' on the MSX matrix.

