# Xmas jumper
This is the code for my Christmas 2015 jumper. The jumper has 6 sets of LEDs arranged around the Christmas trees on the jumper and one on the reindeer's nose.

The code runs of an Atmel ATtiny2313 microcontroller.

The program lights the LEDs according to the five stored modes. The mode can be switched by pressing the button attached to the micro.

Due to the relative large size of the modes array and the small size of the ATtiny's RAM, the modes are stored in program memory.
