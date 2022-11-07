# matrix-kbd-simulator

Matrix keyboard simulator for MSX Sony HB10 and HB20 using an Arduino (@16Mhz).

I got tired of fixing the keyboard membrane, so I started to replace it with a PS2 keyboard.
This is quite done. All I need is to map the PS2 scan codes to the matrix status now.
The hardest part was getting the timing right.
There is a demo running right now. If you leave it connected it will press every key
so you can verify it's working in the MSX screen. One key per second.


# Setup:

You need to connect X0-X7 and Y0 from the keyboard header in your MSX to your Arduino, and this app will simulate to be the keyboard.
Make sure you fix the pinout in the app to your hardware.
I used a Teensy++ 2.0, because it was the first thing I found, so the pins are set for that, but any Arduino at 16Mhz should work.
You will need to fix the timings by yourself using a logic analyzer if you use a different clock.

# Features:

* Simulates all key presses

# THIS IS STILL A WORK IN PROGRESS. TODO:

* Map PS2 keyboard scancodes
* Use the PS2 library from arduino to read from the keyboard
* Create a buffer so keypressed are stored and sent even if the MSX is too slow to read them on time.

# How it works:

When Y0 from the keyboard header goes low, it fires an interrupt in the Arduino. Then the interrupt sets the bits on PORTC to
tell the MSX which keys are pressed or not.
The hardest part is to get the timing right. The Arduino takes 3.5us after the interrupt is fired until it sets the port with
our pressed keys, so I had to experiment with a few workarounds to overcome this problem, since it would skip the entire first
row if I didn't fix the 3.5us delay.
causes all keys from the Y0 column 

I got the idea while trying to fix a broken keyboard membrane, and confirmed it should work when I found someone else who already
did that too:
https://caro-su.translate.goog/msx/kbd4msx.htm?_x_tr_sch=http&_x_tr_sl=auto&_x_tr_tl=en&_x_tr_hl=en&_x_tr_pto=wapp
