
# Bottle to filament

This repository contains all the files used to create the Bottle to filament machine. 



## Environment Variables in the `bottle_to_filament.ino` file

you will need to redefine these variables: 

`#define TERMISTOR_OHM {value}  //defines the resistance of the termistor` 

`#define TARGET_TEMP {value} //defines the desired temperature`

## Pins on the arduino

The thermistor pin is by default assigned to the `A0` analog pin

The heater is by default controled from the pin `9` 


#### All the pins can be rewriten in the `#define` section in the `bottle_to_filament.ino` file. 
