/*

  Programmable Series AC Load Limiter by Bryce Cherry

  Revisions:
  v1.0.0 First release

  Requires the following libraries:
  AdvancedAnalogWrite (version 1.1.0 or later): https://github.com/brycecherry75/AdvancedAnalogWrite
  SimpleClockGenerator (version 1.2.0 or later): http://github.com/brycecherry75/SimpleClockGenerator
  BeyondByte: http://github.com/brycecherry75/BeyondByte
  FieldOps (version 1.0.1 or later): http://github.com/brycecherry75/FieldOps
  SerialFlush: http://github.com/brycecherry75/SerialFlush
  Adafruit GFX
  Adafruit PCD8544 Nokia 5110 LCD with the following modifications described here

  Add the following lines to Adafruit_PCD8544.h:

  In Adafruit_PCD8544.h after the first #define line, add:
  #define PCD8544_HAS_TIMERLESS_DELAY

  In the public section of the same file, add:
  void delayTimerless(word value);

  Add the following at the end of Adafruit_PCD8544.cpp:

  void Adafruit_PCD8544::delayTimerless(word value) {
    for (word i = 0; i < value; i++) {
      delayMicroseconds(1000);
    }
  }

  In the same file, change delay(x) to delayTimerless(x)

  ---

  HOW TO OPERATE:

  Pushbutton presses (will display count from 01-10 in white text on a black background while held):
  < 1 second to reset LCD display if back EMF countermeasures are enabled should back EMF cause the LCD to blank

  If back EMF countermeasures are enabled, one press then release and then a second press within 3 seconds - the load is disabled under this condition (it is impractical to constantly reset the LCD and update the display since it takes 500mS plus data transfer time according to the Adafruit PCD8544 library (later versions have a much shorter delay):
  0-2 seconds to select preset (under diagnostic mode, "OL" will be displayed if the overcurrent comparator senses overcurrent): 3-5 seconds to recall preset, > 5 seconds to exit without recalling a preset and restore the previous preset before this mode was entered
  3-5 seconds to program preset: < 2 seconds to increment digit, 3-5 seconds to change selected digit, > 5 seconds to set current limit and exit
  6-9 seconds to store preset in EEPROM
  10 seconds to assign preset as power on default
  Except under diagnostic mode, the load will be reenabled once normal operation is resumed.

  When powering on, hold the pushbutton and from the time the display shows something, release at:
  2-4 seconds for temporary toggle of EMF countermeasures corresponding to value stored in EEPROM
  5-9 seconds to enter serial programming mode
  10 seconds to enter diagnositc mode/current calibration mode at maximum current with AC output disabled (subsequent pushbutton operation exits to diagnostic mode where load is always disabled) - hold for 5-9 seconds for current sweep test with load disabled, hold for 10 seconds to enable/disable back EMF countermeasures
  Under diagnostic mode, the wavefrom from the mains should not be present or SINEWAVE ONLY error may result.

  If the current does not increment when being programmed, the current with its increment exceeds the current limit; increment the largest digit (and the next digit(s) if necessary) until it decrements to 0

  Error messages displayed on application of power (system halts):
  NO ZERO CROSS: No zero crossing of the mains waveform has been detected at a frequency equal to or higher than half the value of MainsFrequency definition
  SINEWAVE ONLY: A modified sinewave waveform has been detected or mains frequency is 33% higher than the MainsFrequency definition - this unit will only operate with a pure sinewave AC source
  NO ALT CYCLES: Only positive or negative AC cycles have been detected - this unlt will only operate when positive and negative AC cycles have been detected

  Serial commands when serial programming mode has been entered either by pushbutton hold or on a serial terminal prompt (MUST BE ISOLATED FROM MAINS):
  STORE memory_number current_in_mA: Stores a value in a memory location
  STORE memory_number DEFAULT: Assigns a memory as a power on default
  READ memory_number: Reads the value of a stored memory
  READ ALL: Reads the contents of all stored memories
  CALIBRATE: Programs unit for maximum current limit with AC output disabled for calibration
  COMP_TEST current_in_mA: Checks switching threshold of overcurrent comparator
  BACK_EMF (ON/OFF/CHECK): Enable/disable/check if enabled back EMF countermeasures
  CURRENT_SWEEP: Sweeps current until overcurrent is sensed
  EXIT (DIAGNOSTICS): Exit serial programming mode (cannot be reentered without powering off the unit then powering on again while holding the pushbutton down for 5-9 seconds or answering a serial terminal prompt) - DIAGNOSTICS option exits serial programming mode with diagnostics enabled (load is always disabled until power is removed)

*/

#include <AdvancedAnalogWrite.h> // obtain at http://github.com/brycecherry75/AdvancedAnalogWrite
#include <SimpleClockGenerator.h> // obtain at http://github.com/brycecherry75/SimpleClockGenerator
#include <BeyondByte.h> // obtain at http://github.com/brycecherry75/BeyondByte
#include <FieldOps.h> // obtain at http://github.com/brycecherry75/FieldOps
#include <SerialFlush.h> // obtain at http://github.com/brycecherry75/SerialFlush
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h> // needs to be modified to implement delayTimerless() routine with lcd.delayTimerless() changed to delayTimerless()

// change the following as appropriate for your setup
#define MainsFrequency 50 // Hz - leave at 50 for 60 Hz operation
#define CurrentSensingResistor 100 // milliohms
#define MaximumCurrent 10000UL // mA
#define CurrentDigitCount 5 // must correspond with above
#define LargestDigitIncrement 10000UL
#define PWMsteps 65534 // variable resolution PWM with ICR
#define LogicVoltage 5000 // mV
#define PWMfactor 2 // ((LogicVoltage * (MaximumCurrent / PWM steps) * PWMfactor) / 1000) must be greater than the voltage drop of the current sensing resistor at MaximumCurrent
#define PWM_RCfilter_RiseTime 125 // mS - based on PWM RC filter 0-100% rise time * 1.25
#define PWM_RCfilter_RiseTime_PerStep ((1000000UL * 2) / (F_CPU / MaximumCurrent)) // uS based on PWM frequency determined by MaximumCurrent under OCR/ICR mode
#define CurrentPresets 10 // 10 is typically enough for most users

// the following is used for checking for alternate AC cycles
#define MaxZeroCrossingTimeIncrement 10 // uS
#define MaxZeroCrossingTime (((1000000UL * 2) / MainsFrequency) / MaxZeroCrossingTimeIncrement)

#if !defined(PCD8544_HAS_TIMERLESS_DELAY)
#error PCD8544 library must implement and use delayTimerless() instead of delay()
#elif (PWMfactor * MaximumCurrent) > PWMsteps
#error PWMfactor * MaximumCurrent exceeds PWMsteps
#elif (PWMfactor * ((LogicVoltage * MaximumCurrent) / PWMsteps)) < ((MaximumCurrent * CurrentSensingResistor) / 1000)
#error Increase PWMfactor so that (PWMfactor * ((LogicVoltage * MaximumCurrent) / PWMsteps)) is greater than ((MaximumCurrent * CurrentSensingResistor) / 1000) and change the reference divider as necessary
#elif MaxZeroCrossingTime > 65535
#error MainsFrequency is too low
#elif CurrentPresets > 100
#error CurrentPresets exceeds 100
#elif CurrentPresets < 1
#error CurrentPresets must be at least 1
#elif MaximumCurrent > 99999UL
#error MaximumCurrent exceeds 5 digits
#elif (LargestDigitIncrement != 10000 && LargestDigitIncrement != 1000 && LargestDigitIncrement != 100 && LargestDigitIncrement != 10 && LargestDigitIncrement != 1)
#error LargestDigitIncrement must be 1/10/100/1000/10000
#elif ((LargestDigitIncrement == 10000 && CurrentDigitCount != 5) || (LargestDigitIncrement == 1000 && CurrentDigitCount != 4) || (LargestDigitIncrement == 100 && CurrentDigitCount != 3) || (LargestDigitIncrement == 10 && CurrentDigitCount != 2) || (LargestDigitIncrement == 1 && CurrentDigitCount != 1))
#error CurrentDigitCount/LargestDigitIncrement must be 5/10000 or 4/1000 or 3/100 or 2/10 or 1/1
#elif MaximumCurrent > (PWMsteps - 10000)
#error MaximumCurrent exceeds 55534
#endif

const byte DebounceDelay = 50; // mS

// EEPROM
#if PWMsteps <= 255
#define BytesPerPreset 1
#elif PWMsteps <= 65534
#define BytesPerPreset 2
#else
#error PWMsteps exceeds 65534
#endif
#define CurrentPresetBase 0x0000
#define PresetRecallBase (CurrentPresetBase + (CurrentPresets * BytesPerPreset))
#define BackEMFcountermeasuresRequiredBase (PresetRecallBase + 1)

// input pins
const byte ZeroCrossingIRQpin = 2;
const byte OvercurrentIRQpin = 3;
const byte Pushbutton = 5;

// output pins
const byte CurrentLimitPWM = 9;
const byte OutputEnable = 4;
const byte DummyACcycles = 11;
const byte LCD_SCLK = 13;
const byte LCD_DIN = 8;
const byte LCD_DC = 12;
const byte LCD_CS = 7;
const byte LCD_Reset = 6;

// working values
word ZeroCrossingTime; // calculated on mains frequency measured at initialization
word CurrentLimit; // retrieved from EEPROM on initialization
byte RecalledPreset; // retrieved from EEPROM on initialization
bool BackEMFcountermeasuresRequired; // retrieved from EEPROM on initialization
bool DummyZeroCrossingIRQenable = false;
volatile bool ZeroCrossingDetected = false; // true on zero crossing IRQ
volatile bool OvercurrentSensed = false; // true on overcurrent IRQ
volatile byte EnableOutput = LOW; // output is disabled on setup and is HIGH after initialization passes diagnostic tests

// LCD definitions
Adafruit_PCD8544 lcd = Adafruit_PCD8544(LCD_SCLK, LCD_DIN, LCD_DC, LCD_CS, LCD_Reset);
const word TimeToKeepMessaage = 2000; // mS

// LCD - text positions/font sizes
const byte Percentage_FontSize = 3;
const byte Percentage_X = 6;
const byte Percentage_Y = 0;
const byte Percentage_W = 54;
const byte Percentage_H = 24;
const byte PercentageSign_FontSize = 3;
const byte PercentageSign_X = 60;
const byte PercentageSign_Y = 0;
const byte PercentageSign_W = 18;
const byte PercentageSign_H = 24;
const byte Current_FontSize = 2;
const byte Current_X = 6;
const byte Current_Y = 32;
const byte Current_W = 60;
const byte Current_H = 16;
const byte CurrentUnit_FontSize = 1;
const byte CurrentUnit_X = 66;
const byte CurrentUnit_Y = 40;
const byte CurrentUnit_W = 12;
const byte CurrentUnit_H = 8;
const byte Preset_FontSize = 1;
const byte Preset_X = 66;
const byte Preset_Y = 24;
const byte Preset_W = 12;
const byte Preset_H = 8;
const byte Message_FontSize = 1;
const byte Message_X = 6;
const byte Message_Y = 24;
const byte Message_W = 60;
const byte Message_H = 8;
const byte TimePushbuttonHeld_FontSize = 1;
const byte TimePushbuttonHeld_X = 66;
const byte TimePushbuttonHeld_Y = 32;
const byte TimePushbuttonHeld_W = 12;
const byte TimePushbuttonHeld_H = 8;

// LCD - messages
const byte MessageNone = 0;
const byte MessageRecalled = 1;
const byte MessageRecall = 2;
const byte MessageStored = 3;
const byte MessageDefault = 4;
const byte MessageUnits = 5;
const byte MessageTens = 6;
const byte MessageHundreds = 7;
const byte MessageThousands = 8;
const byte MessageTenThousands = 9;
const byte MessageExit = 10;
const byte MessageSetMA = 11;
const byte ErrorNone = 0;
const byte ErrorSinewaveOnly = 1;
const byte ErrorNoAltCycles = 2;
const byte ErrorNoZeroCross = 3;

// ensures that the serial port is flushed fully on request
const unsigned long SerialPortRate = 9600;

const byte commandSize = 25;
const byte FieldSize = 15;

void ZeroCrossingIRQ() {
  ZeroCrossingDetected = true;
  digitalWrite(OutputEnable, EnableOutput);
}

void OvercurrentIRQ() {
  digitalWrite(OutputEnable, LOW);
  OvercurrentSensed = true;
}

void EnableInterrupts() {
  attachInterrupt(digitalPinToInterrupt(OvercurrentIRQpin), OvercurrentIRQ, FALLING); // interrupt on each zero crossing
  attachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin), ZeroCrossingIRQ, CHANGE); // interrupt on each zero crossing
}

void DisableInterrupts() {
  detachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin));
  digitalWrite(OutputEnable, LOW);
  detachInterrupt(digitalPinToInterrupt(OvercurrentIRQpin));
}

void PrintMessage(byte MessageToPrint, bool KeepMessage) {
  lcd.setCursor(Message_X, Message_Y);
  lcd.fillRect(Message_X, Message_Y, Message_W, Message_H, WHITE);
  lcd.setCursor(Message_X, Message_Y);
  lcd.setTextSize(Message_FontSize);
  switch (MessageToPrint) {
    case MessageNone:
      break;
    case MessageRecalled:
      lcd.print(F("RECALLED"));
      break;
    case MessageRecall:
      lcd.print(F("RECALL"));
      break;
    case MessageStored:
      lcd.print(F("STORED"));
      break;
    case MessageDefault:
      lcd.print(F("DEFAULT"));
      break;
    case MessageUnits:
      lcd.print(F("1"));
      break;
    case MessageTens:
      lcd.print(F("10"));
      break;
    case MessageHundreds:
      lcd.print(F("100"));
      break;
    case MessageThousands:
      lcd.print(F("1000"));
      break;
    case MessageTenThousands:
      lcd.print(F("10000"));
      break;
    case MessageExit:
      lcd.print(F("EXIT"));
      break;
    case MessageSetMA:
      lcd.print(F("SET mA"));
      break;
  }
  lcd.display();
  if (KeepMessage == false) {
    lcd.delayTimerless(TimeToKeepMessaage);
    lcd.setCursor(Message_X, Message_Y);
    lcd.fillRect(Message_X, Message_Y, Message_W, Message_H, WHITE);
    lcd.display();
  }
}

word RecallPreset(byte preset) {
  DisableInterrupts();
  word data = BeyondByte.readWord(((preset * BytesPerPreset) + CurrentPresetBase), BytesPerPreset, BeyondByte_EEPROM, MSBFIRST);
  EnableInterrupts();
  return data;
}

void StorePreset(word value, byte preset) {
  DisableInterrupts();
  BeyondByte.writeWord(((preset * BytesPerPreset) + CurrentPresetBase), value, BytesPerPreset, BeyondByte_EEPROM, MSBFIRST);
  EnableInterrupts();
}

void WriteCurrentLimit(bool SweepUsed) {
  word temp = (MaximumCurrent + 1);
  temp -= CurrentLimit; // inverted mode with ICR to avoid unwanted pulse when CurrentLimit = 0
  AdvancedAnalogWrite.write(CurrentLimitPWM, temp, 0);
  if (SweepUsed == false) {
    lcd.delayTimerless(PWM_RCfilter_RiseTime);
  }
  else {
    if (CurrentLimit != 0) {
      delayMicroseconds(PWM_RCfilter_RiseTime_PerStep);
    }
    else { // ensure comparator voltage is at 0 on start due to fall time and clear overcurrent sense flag
      lcd.delayTimerless(PWM_RCfilter_RiseTime);
      OvercurrentSensed = false;
    }
  }
}

void DisplayCurrent(bool PadWithZeros) {
  lcd.setCursor(Current_X, Current_Y);
  lcd.fillRect(Current_X, Current_Y, Current_W, Current_H, WHITE);
  lcd.setCursor(Current_X, Current_Y);
  lcd.setTextSize(Current_FontSize);
  byte ZeroCount = 0;
  if (CurrentLimit < 10) {
    ZeroCount = 4;
  }
  else if (CurrentLimit < 100) {
    ZeroCount = 3;
  }
  else if (CurrentLimit < 1000) {
    ZeroCount = 2;
  }
  else if (CurrentLimit < 10000) {
    ZeroCount = 1;
  }
  if (ZeroCount != 0) {
    for (int i = 0; i < ZeroCount; i++) {
      if (PadWithZeros == true) {
        lcd.print(F("0"));
      }
      else {
        lcd.print(F(" "));
      }
    }
  }
  lcd.print(CurrentLimit);
  lcd.display();
}

void DisplayPreset() {
  lcd.setCursor(Preset_X, Preset_Y);
  lcd.fillRect(Preset_X, Preset_Y, Preset_W, Preset_H, WHITE);
  lcd.setCursor(Preset_X, Preset_Y);
  lcd.setTextSize(Preset_FontSize);
#if CurrentPresets == 100
  lcd.print(RecalledPreset); // only two digits can fit - 0 as a preset has to be used
#else
  lcd.print((RecalledPreset + 1)); // RecalledPreset starts from zero
#endif
  lcd.display();
}

void DisplayLoadPercentage(byte value) {
  lcd.setCursor(Percentage_X, Percentage_Y);
  lcd.fillRect(Percentage_X, Percentage_Y, Percentage_W, Percentage_H, WHITE);
  lcd.setCursor(Percentage_X, Percentage_Y);
  lcd.setTextSize(Percentage_FontSize);
  // pad with spaces if necessary
  if (value < 10) {
    lcd.print(F("  "));
  }
  else if (value < 100) {
    lcd.print(F(" "));
  }
  lcd.print(value);
  lcd.display();
}

bool PushbuttonTimeout(byte value) {
  bool PushbuttonPressTimeout = true;
  for (int i = 0; i <= (value * 10); i++) {
    if (digitalRead(Pushbutton) == LOW) {
      lcd.delayTimerless(DebounceDelay);
      PushbuttonPressTimeout = false;
      break;
    }
    lcd.delayTimerless(100);
  }
  return PushbuttonPressTimeout;
}

byte TimePushbuttonHeld() {
  lcd.setTextColor(WHITE); // distinguish between the preset and the time pushbutton has been held
  lcd.setTextSize(TimePushbuttonHeld_FontSize);
  byte value = 0;
  if (digitalRead(Pushbutton) == LOW) {
    lcd.delayTimerless(DebounceDelay);
    for (int i = 0; i <= 100; i++) {
      if (digitalRead(Pushbutton) == HIGH) {
        break;
      }
      lcd.delayTimerless(100);
      value = i;
      value /= 10;
      if (i > 0 && (i % 10) == 0) { // only update once per second and if count is not zero
        lcd.fillRect(TimePushbuttonHeld_X, TimePushbuttonHeld_Y, TimePushbuttonHeld_W, TimePushbuttonHeld_H, BLACK); // distinguish between the preset and the time pushbutton has been held
        lcd.setCursor(TimePushbuttonHeld_X, TimePushbuttonHeld_Y);
        if (value < 10) { // pad with a leading zero if necessary
          lcd.print(F("0"));
        }
        lcd.print(value);
        lcd.display();
      }
    }
    while (digitalRead(Pushbutton) == LOW) {
    }
    lcd.delayTimerless(DebounceDelay);
  }
  lcd.setTextColor(BLACK); // distinguishing between the preset and the time pushbutton has been held is no longer needed
  lcd.fillRect(TimePushbuttonHeld_X, TimePushbuttonHeld_Y, TimePushbuttonHeld_W, TimePushbuttonHeld_H, WHITE); // blank the time the pushbutton has been held
  lcd.display();
  return value; // in seconds
}

void WaitForZeroCrossingOrTimeout() {
  for (int i = 0; i < MaxZeroCrossingTime; i++) {
    if (ZeroCrossingDetected == true) {
      break;
    }
    delayMicroseconds(MaxZeroCrossingTimeIncrement);
  }
}

void setup() {
  digitalWrite(OutputEnable, LOW);
  pinMode(OutputEnable, OUTPUT);
  pinMode(ZeroCrossingIRQpin, INPUT);
  pinMode(Pushbutton, INPUT_PULLUP);
  pinMode(OvercurrentIRQpin, INPUT_PULLUP);
  pinMode(DummyACcycles, OUTPUT);
  digitalWrite(DummyACcycles, LOW);
  byte ErrorCode = ErrorNone;
  AdvancedAnalogWrite.init(CurrentLimitPWM, (MaximumCurrent + 1), FastPWM_ICR, INVERTED); // eliminate unwanted pulse if PWM value = 0
  AdvancedAnalogWrite.write(CurrentLimitPWM, (MaximumCurrent + 1), 0); // eliminate unwanted pulse if PWM value = 0
  AdvancedAnalogWrite.start(CurrentLimitPWM, PS_NONE);
  lcd.begin();
  lcd.setContrast(60);
  lcd.clearDisplay();
  lcd.setTextSize(1);
  lcd.setTextColor(BLACK);
  BackEMFcountermeasuresRequired = EEPROM.read(BackEMFcountermeasuresRequiredBase);
  if (BackEMFcountermeasuresRequired != true && BackEMFcountermeasuresRequired != false) {
    BackEMFcountermeasuresRequired = false;
    EEPROM.write(BackEMFcountermeasuresRequiredBase, BackEMFcountermeasuresRequired);
  }
  byte ActionToTake = TimePushbuttonHeld();
  lcd.setCursor(0, 0);
  if (ActionToTake == 0 || (ActionToTake >= 5 && ActionToTake <= 9)) {
    Serial.begin(SerialPortRate);
    while (!Serial) { // wait for the serial port to become ready
    }
    if (ActionToTake == 0) { // prompt for serial programming entry via serial terminal if pushbutton not operated
      Serial.print(F("Enter a command within 3 seconds to enter serial programming mode"));
      lcd.print(F("SERIAL PROGRAM"));
      lcd.setCursor(0, 8);
      lcd.print(F("WAIT"));
      lcd.display();
      for (int i = 0; i < 3; i++) {
        Serial.print(F("."));
        lcd.print(F("."));
        lcd.display();
        lcd.delayTimerless(1000);
        if (Serial.available() > 0) {
          break;
        }
      }
      Serial.println(F(""));
      if (Serial.available() > 0) {
        SerialFlush.flushSerial(SerialPortRate);
        ActionToTake = 5;
      }
      else {
        Serial.end();
      }
      lcd.clearDisplay();
      lcd.setCursor(0, 0);
    }
  }
  if (ActionToTake >= 10) { // enter current calibration/diagnostic mode
    CurrentLimit = MaximumCurrent;
    WriteCurrentLimit(false);
    DisableInterrupts();
    lcd.clearDisplay();
    lcd.setCursor(0, 0);
    lcd.print(F("CALIBRATION"));
    lcd.setCursor(0, 8);
    lcd.print(CurrentLimit);
    lcd.print(F(" mA"));
    lcd.setCursor(0, 16);
    lcd.print(F("Adjust VR1 for"));
    unsigned long V = MaximumCurrent; // 10000 mA in this case
    V *= CurrentSensingResistor; // now 10^5 with 100 milliohm resistor
    unsigned long mV = V;
    V /= 1000000UL; // result is 1
    mV %= 1000000UL; // result is 0
    lcd.setCursor(0, 24);
    lcd.print(V);
    lcd.print(F("."));
    lcd.print(mV);
    lcd.print(F(" V"));
    lcd.setCursor(0, 32);
    lcd.print(F("at TP2"));
    lcd.setCursor(0, 40);
    lcd.print(F("Press to exit"));
    lcd.display();
    while (digitalRead(Pushbutton) == HIGH) { // wait for pushbutton press
    }
    lcd.clearDisplay();
    ActionToTake = TimePushbuttonHeld(); // display will be updated here
    lcd.clearDisplay();
    lcd.setCursor(0, 0);
    lcd.print(F("DIAGNOSTIC"));
    lcd.setCursor(0, 8);
    lcd.print(F("MODE ENTERED"));
    if (ActionToTake >= 10) {
      BackEMFcountermeasuresRequired = !BackEMFcountermeasuresRequired;
      EEPROM.write(BackEMFcountermeasuresRequiredBase, BackEMFcountermeasuresRequired);
      lcd.setCursor(0, 16);
      lcd.print(F("BACK EMF"));
      lcd.setCursor(0, 24);
      lcd.print(F("COUNTERMEASURE"));
      lcd.setCursor(0, 32);
      if (BackEMFcountermeasuresRequired == true) {
        lcd.print(F("ENABLED"));
      }
      else {
        lcd.print(F("DISABLED"));
      }
      lcd.display();
      lcd.delayTimerless(5000);
    }
    else if (ActionToTake >= 5 && ActionToTake <= 9) {
      lcd.setCursor(0, 16);
      lcd.print(F("OVERCURRENT"));
      lcd.setCursor(0, 24);
      lcd.print(F("SWEEP: SENSE"));
      lcd.setCursor(0, 32);
      lcd.print(F("AT "));
      lcd.display();
      EnableInterrupts();
      while (true) {
        if (digitalRead(Pushbutton) == LOW) { // press will exit
          break;
        }
        for (word i = 0; i <= MaximumCurrent; i++) {
          if (digitalRead(Pushbutton) == LOW) { // press will exit
            break;
          }
          CurrentLimit = i;
          WriteCurrentLimit(true);
          if (OvercurrentSensed == true) {
            lcd.fillRect(18, 32, 54, 8, WHITE); // width is long enough for "OUT LIMIT" message
            lcd.setCursor(18, 32);
            byte ZeroCount = 0;
            if (CurrentLimit < 10) {
              ZeroCount = 4;
            }
            else if (CurrentLimit < 100) {
              ZeroCount = 3;
            }
            else if (CurrentLimit < 1000) {
              ZeroCount = 2;
            }
            else if (CurrentLimit < 10000) {
              ZeroCount = 1;
            }
            for (int ZerosToPrint = 0; ZerosToPrint < ZeroCount; ZerosToPrint++) { // pad with zeros if necessary
              lcd.print(F("0"));
            }
            lcd.print(CurrentLimit);
            lcd.print(F(" mA"));
            lcd.display();
            break;
          }
          else if (OvercurrentSensed == false && CurrentLimit == MaximumCurrent) {
            lcd.fillRect(18, 32, 54, 8, WHITE); // width is long enought for "OUT LIMIT" message
            lcd.setCursor(18, 32);
            lcd.print(F("OUT LIMIT"));
            lcd.display();
          }
        }
      }
      DisableInterrupts();
      lcd.delayTimerless(DebounceDelay);
      while (digitalRead(Pushbutton) == LOW) { // wait for release
      }
      lcd.delayTimerless(DebounceDelay);
    }
    lcd.setCursor(0, 0);
    lcd.clearDisplay();
    lcd.display();
    DummyZeroCrossingIRQenable = true;
    SimpleClockGenerator.init(DummyACcycles);
    SimpleClockGenerator.start(DummyACcycles, MainsFrequency);
  }
  else if (ActionToTake >= 5 && ActionToTake <= 9) {
    lcd.print(F("SERIAL PROGRAM"));
    lcd.setCursor(0, 8);
    lcd.print(F("MODE ENTERED"));
    lcd.display();
    Serial.println(F("Entering serial programming mode"));
    char command[commandSize];
    byte ByteCount = 0;
    while (true) {
      if (Serial.available() > 0) {
        char inData = Serial.read();
        if (inData != '\n' && ByteCount < commandSize) {
          command[ByteCount] = inData;
          ByteCount++;
        }
        else {
          bool ValidField = true;
          ByteCount = 0;
          if (FieldOps.compareString(commandSize, FieldSize, command, "STORE", 0, 0x20, 0x0D, false) == true) {
            bool StoreDefault = false;
            word MemoryToStore = FieldOps.extractInt(commandSize, FieldSize, command, 1, 0x20, 0x0D);
            word ValueToStore;
            if (FieldOps.compareString(commandSize, FieldSize, command, "DEFAULT", 2, 0x20, 0x0D, false) == true) {
              StoreDefault = true;
            }
            else {
              ValueToStore = FieldOps.extractInt(commandSize, FieldSize, command, 2, 0x20, 0x0D);
            }
            if (MemoryToStore <= CurrentPresets && ValueToStore <= MaximumCurrent && ((CurrentPresets >= 100 && MemoryToStore >= 0 && MemoryToStore < 100) || (CurrentPresets < 100 && MemoryToStore >= 1 && MemoryToStore < 100))) {
              if (CurrentPresets < 100) { // memories start at 1 if below 100 are stored
                MemoryToStore--;
              }
              if (StoreDefault == false) {
                StorePreset(ValueToStore, MemoryToStore);
              }
              else {
                EEPROM.write(PresetRecallBase, MemoryToStore);
              }
            }
            else {
              ValidField = false;
            }
          }
          else if (FieldOps.compareString(commandSize, FieldSize, command, "READ", 0, 0x20, 0x0D, false) == true) {
            if (FieldOps.compareString(commandSize, FieldSize, command, "ALL", 1, 0x20, 0x0D, false) == true) {
              Serial.print(F("Default: "));
              byte ProgrammedDefault = EEPROM.read(PresetRecallBase);
              if (CurrentPresets < 100) { // memories start at 1 if below 100 are stored
                ProgrammedDefault++;
              }
              Serial.println(ProgrammedDefault);
              for (int i = 0; i < CurrentPresets; i++) {
                word PresetToRead = i;
                if (CurrentPresets < 100) { // memories start at 1 if below 100 are stored
                  PresetToRead++;
                }
                Serial.print(F("Memory "));
                Serial.print(PresetToRead);
                Serial.print(F(": "));
                Serial.print(RecallPreset(i));
                Serial.println(F(" mA"));
              }
            }
            else {
              word MemoryToRead = FieldOps.extractInt(commandSize, FieldSize, command, 1, 0x20, 0x0D);
              if (MemoryToRead <= CurrentPresets && ((CurrentPresets >= 100 && MemoryToRead >= 0 && MemoryToRead < 100) || (CurrentPresets < 100 && MemoryToRead >= 1 && MemoryToRead < 100))) {
                if (CurrentPresets < 100) { // memories start at 1 if below 100 are stored
                  MemoryToRead--;
                }
                Serial.print(RecallPreset(MemoryToRead));
                Serial.println(F(" mA"));
              }
              else {
                ValidField = false;
              }
            }
          }
          else if (FieldOps.compareString(commandSize, FieldSize, command, "CALIBRATE", 0, 0x20, 0x0D, false) == true) {
            CurrentLimit = MaximumCurrent;
            WriteCurrentLimit(false);
            DisableInterrupts();
            Serial.print(F("Adjust VR1 for "));
            unsigned long V = MaximumCurrent; // 10000 mA in this case
            V *= CurrentSensingResistor; // now 10^5 with 100 milliohm resistor
            unsigned long mV = V;
            V /= 1000000UL; // result is 1
            mV %= 1000000UL; // result is 0
            Serial.print(V);
            Serial.print(F("."));
            Serial.print(mV);
            Serial.println(F("V at TP2"));
          }
          else if (FieldOps.compareString(commandSize, FieldSize, command, "COMP_TEST", 0, 0x20, 0x0D, false) == true) {
            word CurrentToTest = FieldOps.extractInt(commandSize, FieldSize, command, 1, 0x20, 0x0D);
            if (CurrentToTest <= MaximumCurrent) {
              Serial.println(F("Entering overcurrent comparator test"));
              CurrentLimit = CurrentToTest;
              WriteCurrentLimit(false);
              SerialFlush.flushSerial(SerialPortRate);
              OvercurrentSensed = false;
              EnableInterrupts();
              while (true) {
                if (OvercurrentSensed == true || digitalRead(OvercurrentIRQpin) == LOW) {
                  Serial.print(F("Current is now greater than programmed threshold - "));
                  if (OvercurrentSensed == true) {
                    OvercurrentSensed = false;
                    Serial.println(F("interrupt"));
                  }
                  else {
                    Serial.println(F("poll"));
                  }
                  while (digitalRead(OvercurrentIRQpin) == LOW && Serial.available() == 0) {
                  }
                }
                if (Serial.available() > 0) {
                  Serial.println(F("End of overcurrent comparator test"));
                  break;
                }
                lcd.delayTimerless(1000);
              }
              DisableInterrupts();
            }
            else {
              ValidField = false;
            }
          }
          else if (FieldOps.compareString(commandSize, FieldSize, command, "BACK_EMF", 0, 0x20, 0x0D, false) == true) {
            bool EEPROMwriteRequired = true;
            if (FieldOps.compareString(commandSize, FieldSize, command, "ON", 1, 0x20, 0x0D, false) == true) {
              BackEMFcountermeasuresRequired = true;
            }
            else if (FieldOps.compareString(commandSize, FieldSize, command, "OFF", 1, 0x20, 0x0D, false) == true) {
              BackEMFcountermeasuresRequired = false;
            }
            else if (FieldOps.compareString(commandSize, FieldSize, command, "CHECK", 1, 0x20, 0x0D, false) == true) {
              EEPROMwriteRequired = false;
              Serial.print(F("Back EMF countermeasures "));
              if (BackEMFcountermeasuresRequired == true) {
                Serial.println(F("enabled"));
              }
              else {
                Serial.println(F("disabled"));
              }
            }
            else {
              ValidField = false;
              EEPROMwriteRequired = false;
            }
            if (EEPROMwriteRequired == true) {
              EEPROM.write(BackEMFcountermeasuresRequiredBase, BackEMFcountermeasuresRequired);
            }
          }
          else if (FieldOps.compareString(commandSize, FieldSize, command, "CURRENT_SWEEP", 0, 0x20, 0x0D, false) == true) {
            SerialFlush.flushSerial(SerialPortRate);
            EnableInterrupts();
            while (true) {
              if (Serial.available() > 0) {
                break;
              }
              for (word i = 0; i <= MaximumCurrent; i++) {
                if (Serial.available() > 0) {
                  break;
                }
                CurrentLimit = i;
                WriteCurrentLimit(true);
                if (OvercurrentSensed == true) {
                  Serial.print(F("Overcurrent sensed at "));
                  Serial.print(CurrentLimit);
                  Serial.println(F(" mA"));
                  break;
                }
                else if (OvercurrentSensed == false && CurrentLimit == MaximumCurrent) {
                  Serial.println(F("Overcurrent sense is not within limits"));
                }
              }
            }
            DisableInterrupts();
          }
          else if (FieldOps.compareString(commandSize, FieldSize, command, "EXIT", 0, 0x20, 0x0D, false) == true) {
            if (FieldOps.compareString(commandSize, FieldSize, command, "DIAGNOSTICS", 1, 0x20, 0x0D, false) == true) {
              Serial.println(F("Diagnostic mode enabled"));
              DummyZeroCrossingIRQenable = true;
              SimpleClockGenerator.init(DummyACcycles);
              SimpleClockGenerator.start(DummyACcycles, MainsFrequency);
            }
            Serial.println(F("Exiting serial programming mode - to reenter, power off then while holding the pushbutton, power on and hold the pushbutton for 5-9 seconds or enter anything on a serial terminal prompt"));
            break;
          }
          else {
            ValidField = false;
          }
          if (ValidField == true) {
            Serial.println(F("OK"));
          }
          else {
            Serial.println(F("ERROR"));
          }
          SerialFlush.flushSerial(SerialPortRate);
        }
      }
    }
    Serial.end();
    lcd.setCursor(0, 0);
    lcd.clearDisplay();
    lcd.display();
  }
  else if (ActionToTake >= 2 && ActionToTake <= 4) {
    BackEMFcountermeasuresRequired = !BackEMFcountermeasuresRequired;
    lcd.setCursor(0, 0);
    lcd.print(F("BACK EMF"));
    lcd.setCursor(0, 8);
    lcd.print(F("COUNTERMEASURE"));
    lcd.setCursor(0, 16);
    lcd.print(F("TEMPORARY"));
    lcd.setCursor(0, 24);
    if (BackEMFcountermeasuresRequired == true) {
      lcd.print(F("ENABLE"));
    }
    else {
      lcd.print(F("DISABLE"));
    }
    lcd.display();
    lcd.delayTimerless(5000);
    lcd.setCursor(0, 0);
    lcd.clearDisplay();
    lcd.display();
  }
  for (int i = 0; i < CurrentPresets; i++) { // check if stored presets are within a valid range and zero if invalid
    word PresetCurrent = RecallPreset(i);
    if (PresetCurrent > MaximumCurrent) {
      PresetCurrent = 0;
      StorePreset(PresetCurrent, i);
    }
  }
  if (EEPROM.read(PresetRecallBase) >= CurrentPresets) { // correct the value to the first preset if necessary
    EEPROM.write(PresetRecallBase, 0);
  }
  WaitForZeroCrossingOrTimeout();
  if (ZeroCrossingDetected == true) {
    unsigned long StartTime;
    unsigned long StopTime;
    bool BothEdgesDetected = true;
    detachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin));
    ZeroCrossingDetected = false;
    attachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin), ZeroCrossingIRQ, RISING); // interrupt on rising edge only to detect modified sinewave - only positive modified sinewave cycle time can be measured
    WaitForZeroCrossingOrTimeout();
    ZeroCrossingDetected = false;
    WaitForZeroCrossingOrTimeout();
    if (ZeroCrossingDetected == false) {
      BothEdgesDetected = false;
    }
    else {
      detachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin));
      ZeroCrossingDetected = false;
      attachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin), ZeroCrossingIRQ, FALLING); // interrupt on falling edge only to detect modified sinewave - only positive modified sinewave cycle time can be measured
      WaitForZeroCrossingOrTimeout();
      ZeroCrossingDetected = false;
      StartTime = micros();
      WaitForZeroCrossingOrTimeout();
      if (ZeroCrossingDetected == false) {
        BothEdgesDetected = false;
      }
      StopTime = micros();
      detachInterrupt(digitalPinToInterrupt(ZeroCrossingIRQpin));
      StopTime -= StartTime; // deals with micros() overflow
      ZeroCrossingTime = StopTime;
    }
    if (BothEdgesDetected == true && ZeroCrossingTime < (((1000000UL / MainsFrequency / 2) * 75) / 100)) { // modified sinewave is detected
      ErrorCode = ErrorSinewaveOnly;
    }
    else if (BothEdgesDetected == false) {
      ErrorCode = ErrorNoAltCycles;
    }
  }
  else {
    ErrorCode = ErrorNoZeroCross;
  }
  DisableInterrupts();
  if (ErrorCode != ErrorNone) {
    if (ErrorCode == ErrorSinewaveOnly) {
      lcd.print(F("SINEWAVE ONLY"));
    }
    else if (ErrorCode == ErrorNoAltCycles) {
      lcd.print(F("NO ALT CYCLES"));
    }
    else if (ErrorCode == ErrorNoZeroCross) {
      lcd.print(F("NO ZERO CROSS"));
    }
    lcd.display();
    while (true) { // halt
    }
  }
  RecalledPreset = EEPROM.read(PresetRecallBase);
  CurrentLimit = RecallPreset(RecalledPreset);
  EnableInterrupts();
  WriteCurrentLimit(false);
  lcd.clearDisplay();
  lcd.setCursor(CurrentUnit_X, CurrentUnit_Y);
  lcd.print(F("mA"));
  lcd.setCursor(PercentageSign_X, PercentageSign_Y);
  lcd.setTextSize(PercentageSign_FontSize);
  lcd.print(F("%"));
  DisplayCurrent(false); // lcd.display() is performed by this routine
  DisplayPreset();
  if (DummyZeroCrossingIRQenable == false) { // not in diagnostic mode
    EnableOutput = HIGH;
  }
}

void loop() {
  if (ZeroCrossingDetected == true) { // update duty cycle percentage for load
    ZeroCrossingDetected = false;
    unsigned long ZeroCrossingStart = micros();
    while (true) {
      bool PercentageUpdated = false;
      if (digitalRead(Pushbutton) == LOW) {
        lcd.delayTimerless(DebounceDelay);
        break;
      }
      if (OvercurrentSensed == true) {
        unsigned long LoadPercentage = micros();
        LoadPercentage -= ZeroCrossingStart; // 9000uS in this example - deals with overflow
        LoadPercentage *= 100; // now 900000
        LoadPercentage /= ZeroCrossingTime; // result is 90% with 10000uS (50 Hz) per half cycle
        if (LoadPercentage > 100) { // deal with possible overflow
          LoadPercentage = 100;
        }
        DisplayLoadPercentage(LoadPercentage);
        PercentageUpdated = true;
      }
      else if (ZeroCrossingDetected == true) {
        DisplayLoadPercentage(100);
        PercentageUpdated = true;
      }
      if (PercentageUpdated == true) { // percentage update delay (once per second) for the human senses
        for (int i = 0; i < 1000; i++) {
          lcd.delayTimerless(1);
          if (digitalRead(Pushbutton) == LOW) {
            lcd.delayTimerless(DebounceDelay);
            break;
          }
        }
        ZeroCrossingDetected = false;
        OvercurrentSensed = false;
        while (ZeroCrossingDetected == false && digitalRead(Pushbutton) == HIGH) {
          OvercurrentSensed = false;
        }
        break;
      }
    }
  }
  if (digitalRead(Pushbutton) == LOW) {
    byte WhatToDo = 11; // in seconds - with back EMF countermeasures enabled, no second press within 3 seconds will return to normal operating mode
    if (BackEMFcountermeasuresRequired == true) {
      lcd.delayTimerless(DebounceDelay);
      while (digitalRead(Pushbutton) == LOW) { // wait for pushbutton release
      }
      lcd.delayTimerless(DebounceDelay);
      // reset LCD should back EMF occur when disabling load causing the LCD to blank
      lcd.initDisplay();
      lcd.display();
      if (PushbuttonTimeout(3) == false) { // return to normal operating mode on timeout
        EnableOutput = LOW; // disable load to prevent back EMF
        lcd.delayTimerless(DebounceDelay);
        lcd.initDisplay();
        lcd.display();
        WhatToDo = TimePushbuttonHeld(); // in seconds
      }
    }
    else {
      WhatToDo = TimePushbuttonHeld(); // in seconds
    }
    while (true) {
      if (WhatToDo >= 0 && WhatToDo <= 2) { // select preset
        byte OldPreset = RecalledPreset;
        word OldPWMvalue = CurrentLimit;
        while (true) {
          PrintMessage(MessageRecall, true);
          RecalledPreset++;
          if (RecalledPreset >= CurrentPresets) {
            RecalledPreset = 0;
          }
          CurrentLimit = RecallPreset(RecalledPreset);
          DisplayCurrent(false);
          DisplayPreset();
          lcd.setTextColor(WHITE);
          while (true) { // wait for pushbutton press
            if (DummyZeroCrossingIRQenable == true) { // check for comparator overcurrent condition under diagnostic mode and display it
              byte BackgroundColour = WHITE;
              if (OvercurrentSensed == true) {
                BackgroundColour = BLACK;
              }
              lcd.fillRect(TimePushbuttonHeld_X, TimePushbuttonHeld_Y, TimePushbuttonHeld_W, TimePushbuttonHeld_H, BackgroundColour);
              if (OvercurrentSensed == true) {
                OvercurrentSensed = false;
                lcd.setCursor(TimePushbuttonHeld_X, TimePushbuttonHeld_Y);
                lcd.print(F("OL"));
              }
              for (int i = 0; i < 10; i++) {
                if (digitalRead(Pushbutton) == LOW) {
                  lcd.delayTimerless(DebounceDelay);
                  break;
                }
                lcd.delayTimerless(100);
              }
              lcd.display();
            }
            if (digitalRead(Pushbutton) == LOW) {
              break;
            }
          }
          WhatToDo = TimePushbuttonHeld(); // overload status will be blanked and font will be BLACK afterwards
          if (WhatToDo > 2 && WhatToDo <= 5) {
            PrintMessage(MessageRecalled, false);
            WriteCurrentLimit(false);
            WhatToDo = 11;
            break;
          }
          else if (WhatToDo > 5) { // exit
            PrintMessage(MessageExit, false);
            RecalledPreset = OldPreset;
            CurrentLimit = OldPWMvalue;
            DisplayPreset();
            WhatToDo = 11;
            break;
          }
        }
      }
      else if (WhatToDo > 2 && WhatToDo <= 5) { // program preset
        PrintMessage(MessageSetMA, false);
        byte SelectedDigit = 0;
        word DigitIncrement = LargestDigitIncrement;
        while (true) {
          DisplayCurrent(true);
          byte UnitToPrint = MessageUnits;
          if (DigitIncrement == 10000) {
            UnitToPrint = MessageTenThousands;
          }
          else if (DigitIncrement == 1000) {
            UnitToPrint = MessageThousands;
          }
          else if (DigitIncrement == 100) {
            UnitToPrint = MessageHundreds;
          }
          else if (DigitIncrement == 10) {
            UnitToPrint = MessageTens;
          }
          PrintMessage(UnitToPrint, true);
          uint8_t DigitTemp[CurrentDigitCount];
          word factor = LargestDigitIncrement;
          word temp = CurrentLimit;
          for (int i = 0; i < CurrentDigitCount; i++) { // clear the digits
            DigitTemp[i] = 0;
          }
          for (int i = 0; i < CurrentDigitCount; i++) { // convert a single number into places
            while (true) {
              if (temp >= factor) {
                temp -= factor;
                DigitTemp[i]++;
              }
              else {
                factor /= 10;
                break;
              }
            }
          }
          while (digitalRead(Pushbutton) == HIGH) { // wait for pushbutton press
          }
          WhatToDo = TimePushbuttonHeld();
          if (WhatToDo <= 2) { // increment digit
            DigitTemp[SelectedDigit]++;
            if (DigitTemp[SelectedDigit] > 9) { // roll over to zero instead of carrying over
              DigitTemp[SelectedDigit] = 0;
            }
            while (true) { // convert places into a single number
              CurrentLimit = 0;
              word factor = LargestDigitIncrement;
              for (int i = 0; i < CurrentDigitCount; i++) {
                word FigureToAdd = DigitTemp[i];
                FigureToAdd *= factor;
                CurrentLimit += FigureToAdd;
                factor /= 10;
              }
              if (CurrentLimit > MaximumCurrent) {
                DigitTemp[SelectedDigit] = 0;
              }
              else {
                break;
              }
            }
          }
          else if (WhatToDo > 2 && WhatToDo <= 5) { // change digit
            SelectedDigit++;
            DigitIncrement /= 10;
            if (SelectedDigit >= CurrentDigitCount) {
              SelectedDigit = 0;
              DigitIncrement = LargestDigitIncrement;
            }
          }
          else if (WhatToDo > 5) { // exit
            PrintMessage(MessageExit, false);
            WriteCurrentLimit(false);
            WhatToDo = 11;
            break;
          }
        }
      }
      else if (WhatToDo > 5 && WhatToDo <= 9) { // store preset
        PrintMessage(MessageStored, false);
        StorePreset(CurrentLimit, RecalledPreset);
        break;
      }
      else if (WhatToDo == 10) { // assign preset as power on default
        PrintMessage(MessageDefault, false);
        DisableInterrupts();
        EEPROM.write(PresetRecallBase, RecalledPreset);
        EnableInterrupts();
        break;
      }
      else { // WhatToDo = 11
        DisplayCurrent(false);
        break;
      }
    }
    if (BackEMFcountermeasuresRequired == true) {
      if (DummyZeroCrossingIRQenable == false) { // not in diagnostic mode
        EnableOutput = HIGH;
      }
    }
    ZeroCrossingDetected = false;
    OvercurrentSensed = false;
    while (ZeroCrossingDetected == false) {
      OvercurrentSensed = false;
    }
  }
}
