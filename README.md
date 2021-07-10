# SeriesLoadLimiter

## Revisions:

v1.0.0 First release

## Features

+ Displays load duty cycle every second
+ Diagnostic mode entry for testing including calibration of current preset VR1 without load enabled
+ Current presets and power on default can be programmed along with calibration mode being called via a serial terminal (UNIT MUST BE ISOLATED FROM THE MAINS)
+ 10 presets with default power on setting
+ On initialization, senses non-sinewave AC input, AC input without alternate cycles and lack of zero cycle crossings and halts if any one of these conditions is sensed (these checks can be bypassed without load enabled under diagnostic mode)

## How to use

Instructions and requirements are included in the .ino sketch which includes required modifications to the Adafruit PCD8544 Nokia 5110 LCD library.
Requires the following libraries:
AdvancedAnalogWrite (version 1.1.0 or later): https://github.com/brycecherry75/AdvancedAnalogWrite
SimpleClockGenerator (version 1.2.0 or later): http://github.com/brycecherry75/SimpleClockGenerator
BeyondByte: http://github.com/brycecherry75/BeyondByte
FieldOps (version 1.0.1 or later): http://github.com/brycecherry75/FieldOps
SerialFlush: http://github.com/brycecherry75/SerialFlush

The circuit is in a PDF file and the schematic file is in Eagle format.

## Circuit description

Since the availability of suitable filament lamps is quite limited, I came up with a more flexible method of current limiting an AC load.

This unit works on the principle of a trailing edge dimmer which turns on MOSFET Q1 at the zero crossing of the mains waveform and turns off Q1 an overcurrent condition is sensed anywhere before the next zero crossing.

Resistor R6 and capacitor C3 converts a PWM output to an analog voltage for overcurrent threshold comparison by IC3 which is calibrated with R9 and VR1; to trigger on the zero crossing of the mains waveform, R19/R20 and transistors Q2/Q3 form a high sensitivity amplifier which its output will sink when the mains waveform becomes positive with diode D2 limiting negative biasing of Q2 and dummy AC cycles for testing without an AC source are generated with Q4.

Bridge rectifier BR1 is wired in series with the load and switching on Q1 shorts out BR1 which in turn enables power to the load with current being sensed by R12 with this output being connected to the other input of the internal microcontroller analog comparator; to drive Q1, opamp IC2 is wired as a comparator with the rising ON threshold set by R14/R15.

For protection, ZD3 will cause resistor R16 to burn out if Q1 becomes shorted; additionally, the combination of SCR1/ZD4/R22/D4 will cause R13 to burn out if R12 becomes open circuited and C4/R22/D3 along with C5/D3 and TVS1/TVS2 as well as ZD1/ZD2 form countermeasures against back EMF although it cannot be avoided entirely where it can cause the LCD to blank which can be refreshed by pressing the button once with back EMF countermeasures enabled should this happen.

It is essential that this project be built in an insulated case with switches S1 and S2 being non-metallic; during adjustment of VR1 and VR2 under calibration mode with the unit isolated from the mains, apply a +/- 12V 200mA source to TP8/TP9/TP10 (while observing polarity) for accuracy of this calibration considering the voltage drop in USB leads when connected for programming/testing.

Appropriate heatsinking of Q1 and BR1 is also required.