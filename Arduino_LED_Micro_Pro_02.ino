// NOTES:
// This program is for an Arduino Pro Micro board.
// For Board layout, see: https://cdn.sparkfun.com/assets/9/c/3/c/4/523a1765757b7f5c6e8b4567.png
// AND:
// https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/hardware-overview-pro-micro

// ***************************************************************************
// * PROGRAM DESCRIPTION:
// * 
// * 
// ***************************************************************************


// INCLUDES:
// INCLUDE A LIBRARY FOR HANDLING "Neopixel" LED's:
#include <FastLED.h>
// FastLED library located here:   https://github.com/FastLED/FastLED/releases

// INCLUDE THE ARDUINO LIBRARY FOR HANDLING LIQUID CRYSTAL DISPLAYS (LCD's):
#include <LiquidCrystal.h>


// CONSTANTS:
// DEFINE THE ARDUINO PINS:
#define arPin_0 0    // TX Line (USB data going out)
#define arPin_1 1    // RX Line (USB data coming in)
#define arPin_2 2    // Digital In/Out + SDA I2C
#define arPin_3 3    // Digital In/Out + SCL I2C + PWM Out
#define arPin_4 4    // Digital In/Out + Analog In (A6)
#define arPin_5 5    // Digital In/Out + PWM Out
#define arPin_6 6    // Digital In/Out + Analog In (A7) + PWM Out
#define arPin_7 7    // Digital In/Out
#define arPin_8 8    // Digital In/Out + Analog In (A8)
#define arPin_9 9    // Digital In/Out + Analog In (A9) + PWM Out
#define arPin_10 10  // Digital In/Out + Analog In (A10) + PWM Out
#define arPin_14 14  // Digital In/Out + MISO SPI
#define arPin_15 15  // Digital In/Out + SCLK SPI
#define arPin_16 16  // Digital In/Out + MOSI SPI
#define arPin_18 18  // Digital In/Out + Analog In (A0)
#define arPin_19 19  // Digital In/Out + Analog In (A1)
#define arPin_20 20  // Digital In/Out + Analog In (A2)
#define arPin_21 21  // Digital In/Out + Analog In (A3)
// arPin_RAW = Input voltage (5 --> 12V)
// arPin_GND = Ground
// arPin_RST = Reset pin (Drop this pin to ground to reset the chip)
// arPin_VCC = If arPin_RAW powered, then use VCC as +5V power output

// DEFINE WHAT THE LCD PINS ARE CONNECTED TO:
// THE LCD/ARDUINO CIRCUIT: http://www.arduino.cc/en/Tutorial/LiquidCrystal
// #define lcd_GND = Ground         // GND is pin 1 on LCD
// #define lcd_VDD = +5V Power In   // VDD is pin 2 on LCD
// #define lcd_V0 = 0-->+5V In      // V0 is pin 3 on LCD - Supplied by Potentiometer
// NOTE: Place a wire from LCD pin 3 to arPin10 and define arPin10 to receive
//       analog input.
#define lcd_RS arPin_14           // RS = Register Select, and is pin 4 on LCD
// #define lcd_RW Ground          // RW = Read/Write Mode, and is pin 5 on LCD
#define lcd_Enable arPin_16       // Enable is pin 6 on LCD
// #define lcd_DB0 nothing        // DB0 is pin 7 on LCD  // DB = Data Bus
// #define lcd_DB1 nothing        // DB1 is pin 8 on LCD
// #define lcd_DB2 nothing        // DB2 is pin 9 on LCD
// #define lcd_DB3 nothing        // DB3 is pin 10 on LCD
#define lcd_DB4 arPin_18          // DB4 is pin 11 on LCD
#define lcd_DB5 arPin_19          // DB5 is pin 12 on LCD
#define lcd_DB6 arPin_20          // DB6 is pin 13 on LCD
#define lcd_DB7 arPin_21          // DB7 is pin 14 on LCD
// LCD NOTES:
// * The LCD RS pin is the register select pin. It controls where in the LCD's memory
//   you're writing data to. You can select either the data register, which holds
//   what goes on the screen, or an instruction register, which is where the LCD's
//   controller looks for instructions on what to do next.
// * The Read/Write (R/W) pin selects reading mode or writing mode.
// * The Enable pin enables writing to the registers.
// * There are 8 LCD data pins (D0 --> D7).
//   The states of these pins (high or low) are the bits that you're writing to a
//   register when you write, or the values you're reading when you read.
// * 10K Potentiometer (Pot):
//     Pot Left pin to +5V.
//     Pot Right pin to ground (GND).
//     Pot Middle pin (wiper or output pin - where voltage varies from 0-->+5V) to LCD V0 pin (pin 3).

// INITIALIZE THE LCD LIBRARY WITH THE ARDUINO PIN NUMBERS:
LiquidCrystal lcd(lcd_RS, lcd_Enable, lcd_DB4, lcd_DB5, lcd_DB6, lcd_DB7);

// CONSTANTS - NEOPIXELS:
// How many LED's are in your Neopixel LED strip?
#define NUM_leds 56

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define PIN_NEOPIXEL_LED_DATA_LINE.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806, define both PIN_NEOPIXEL_LED_DATA_LINE and CLOCK_PIN.
// Define the Pin on the Pro Micro board that the LED data line is connected to:
#define PIN_NEOPIXEL_LED_DATA_LINE arPin_4
// Define the Pin on the Pro Micro board that the LED CLOCK_PIN is connected to:
// #define CLOCK_PIN 13 -- NOT USED

// Set up the block of memory that will be used for storing and manipulating the led data:
// This creates an array, called "ledArray", of size NUM_leds:
CRGB ledArray[NUM_leds];


// CONSTANTS - DEFINE PULSE WIDTH MODULATION (PWM):
// http://www.camelsoftware.com/firetail/blog/radio/reading-pwm-signals-from-a-remote-control-receiver-with-arduino/
// It's possible to read PWM signals using hardware interrupts. A hardware interrupt is a
// signal that is generated by the hardware that literally interrupts the processor.
// With Arduino, hardware interrupts can be generated by a pin changing value, going LOW,
// going HIGH, rising or falling.
// The processor responds to interrupts by suspending its current activity and handling
// the interrupt with an interrupt handler. After the interrupt handler has returned,
// the processor resumes its previous activity.

// Unlike the easy way, reading PWM inputs with interrupts allows the processor to continue
// with other tasks except for that very brief moment when an interrupt is handled.
// To read PWM inputs we must know when a pin goes HIGH and LOW, and so we are only really
// interested in CHANGE interrupts. When a PWM pin goes HIGH, a timer is started.
// When the pin goes LOW, we can measure the pulse time by checking how much time has passed.

// Arduino has the function attachInterrupt(), which allows us to supply an interrupt handler
// for a particular event and pin number.

// The micros() function Returns the number of microseconds since the Arduino board began
// running the current program. This number will overflow (go back to zero), after
// approximately 70 minutes.

// NOTE:
// The Pro Micro has five external interrupts, which allow you to instantly trigger a function
// when a pin goes either high or low (or both). If you attach an interrupt to an
// interrupt-enabled pin, you will need to know the specific interrupt that pin triggers:
// pin 3 maps to interrupt 0 (I.e. Put the PWM signal on pin 3, and pin 0 will be interupted),
// pin 2 maps to interrupt 1,
// pin 0 maps to interrupt 2,
// pin 1 maps to interrupt 3, and
// pin 7 maps to interrupt 4.

// Define the arduino pin that is receiving the Pulse Width Modulation (PWM) signal:
#define PIN_PWM_SOURCE arPin_7 // Which causes an interrupt on Pin 4 (PIN_NEOPIXEL_LED_DATA_LINE).

// "timer_start" defines the time when the the PIN_PWM_SOURCE pin goes from LOW to HIGH:
volatile unsigned long timer_start;

// "pulse_time" is the length of time that the pin is HIGH:
volatile int pulse_time;

// This is the time that the last interrupt occurred.
// You can use this to determine if your receiver has a signal or not.
volatile int last_interrupt_time;


// CONSTANTS - FLOODLIGHT:
#define PIN_FLOODLIGHT_SWITCH arPin_5 // Define the pin that turns the floodlight off & on.

// CONSTANTS - LOOP() DELAY TIME:
#define delayTime 100  // Delay time constant (1000 = 1 second)


// CONSTANTS - MATH:
int randomNumber = 1;
int val0 = 0;
int val4 = 0;
int val8 = 0;

// CONSTANTS - CYLON EYE:
unsigned char leftRight = 1; // start off going to the RIGHT
unsigned char centre_Pos_Cylon_Eye = (int)round(NUM_leds / 2);  // Centre position of Cylon Eye


// FUNCTIONS:
// SETUP FUNCTION():
void setup()
{
  // This tells the library that there's a strand of NEOPIXEL's on pin PIN_NEOPIXEL_LED_DATA_LINE:
  // and that these LED's will use the LED array: "ledArray"
  FastLED.addLeds<NEOPIXEL, PIN_NEOPIXEL_LED_DATA_LINE>(ledArray, NUM_leds);

  // Initialise serial output (Open the serial port at 115200 bps):
  Serial.begin(115200);
  Serial.print("Hello world!");  // prints hello with ending line break

  // SETUP THE PINS:
  // Set up the digital pin that receives the PWM source(PIN_PWM_SOURCE):
  pinMode(PIN_PWM_SOURCE, INPUT);
  // Set the digital pin 9 (PIN_FLOODLIGHT_SWITCH) as output:
  pinMode(PIN_FLOODLIGHT_SWITCH, OUTPUT);

  // START A TIMER:
  timer_start = 0;

  // Create an "attachInterrupt" so as that when the pin with the PWM source changes,
  // the calcsignal() function is called (I.e. Attach the interrupt handler):
  attachInterrupt(PIN_NEOPIXEL_LED_DATA_LINE, calcSignal, CHANGE);

  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
}


// LOOP FUCNTION():
// The primary loop:
void loop()
{
  // If you want your program to change "functions" based on the "pulse_time" value, then use:
  // if (pulse_time < value) // where value = 1000 --> 2000 microSeconds
  // {
  //    Do_Function_01();
  // }
  // else
  // {
  //    Do_Function_02();
  // }

  // But in this case, we just call the one function:
  cylonEye_02();

  // Update the LED's:
  FastLED.show();

  // Set the loop Update time:
  delay(0.5 * delayTime);
}


// The function "calcSignal()" is the interrupt handler:
void calcSignal()
{
  // NOTE:
  // Your in this function because the Pro Micro Arduino board recorded that there was a
  // change on the Pulse Width Modulation signal coming from pin PIN_PWM_SOURCE.

  // The micros() function Returns the number of microseconds since the Arduino board began
  // running the current program. This number will overflow (go back to zero), after
  // approximately 70 minutes.

  // Record the current board time:
  last_interrupt_time = micros();

  // If the pin PIN_PWM_SOURCE has gone "HIGH", set "timer_start":
  if (digitalRead(PIN_PWM_SOURCE) == HIGH)
  {
    timer_start = micros();
  }
  // Otherwise, the pin has gone "LOW":
  else
  {
    // Only worry about this if the timer has actually started
    if (timer_start > 0)
    {
      // Record the PWM "pulse_time" duration:
      pulse_time = ((volatile int)micros() - timer_start);

      // Reset the timer:
      timer_start = 0;
    }
  }
  //       Serial.println("last_interrupt_time:");
  //       Serial.println(last_interrupt_time);
  //       Serial.println("timer_start:");
  //       Serial.println(timer_start);
  //       Serial.println("pulse_time:");
  //       Serial.println(pulse_time);
}


void cylonEye_02()
{
  // We display a moving cylon eye, and use the PWM signal to continuously change
  // the colour of the "eye":

  // First, clear the existing led values
  FastLED.clear();

  // Note: "centre_Pos_Cylon_Eye" always starts in the middle of the NEO PIXEL array.
  if (leftRight == 1) // If true, then Cylon slash needs to move to the right
  {
    centre_Pos_Cylon_Eye++;
    if (centre_Pos_Cylon_Eye >= (NUM_leds - 1))
    {
      leftRight = 0;
    }
  }
  else
  {
    centre_Pos_Cylon_Eye--;
    if (centre_Pos_Cylon_Eye == 0)
    {
      leftRight = 1;
    }
  }

  // Now set the LED VALUES:
  if (pulse_time < 1256)
  {
    // Adjust pulse_time if less than 1000:
    if (pulse_time < 1000)
    {
      pulse_time = 1000;
    }
    ledArray[centre_Pos_Cylon_Eye - 2].r = (pulse_time - 1000) / 8;
    ledArray[centre_Pos_Cylon_Eye - 1].r = (pulse_time - 1000) / 4;
    ledArray[centre_Pos_Cylon_Eye].r = pulse_time - 1000;
    ledArray[centre_Pos_Cylon_Eye + 1].r = (pulse_time - 1000) / 4;
    ledArray[centre_Pos_Cylon_Eye + 2].r = (pulse_time - 1000) / 8;
    
    // Set the Floodlight Diode Pin to "OFF":
    digitalWrite(PIN_FLOODLIGHT_SWITCH, LOW);
  }
  if ( (pulse_time >= 1256) && (pulse_time < 1512) )
  {
    ledArray[centre_Pos_Cylon_Eye - 2].g = (pulse_time - 1256) / 8;
    ledArray[centre_Pos_Cylon_Eye - 1].g = (pulse_time - 1256) / 4;
    ledArray[centre_Pos_Cylon_Eye].g = pulse_time - 1256;
    ledArray[centre_Pos_Cylon_Eye + 1].g = (pulse_time - 1256) / 4;
    ledArray[centre_Pos_Cylon_Eye + 2].g = (pulse_time - 1256) / 8;

    // Set the Floodlight Diode Pin to "ON":
    digitalWrite(PIN_FLOODLIGHT_SWITCH, HIGH);
  }
  if ( (pulse_time >= 1512) && (pulse_time < 1768) )
  {
    ledArray[centre_Pos_Cylon_Eye - 2].b = (pulse_time - 1512) / 8;
    ledArray[centre_Pos_Cylon_Eye - 1].b = (pulse_time - 1512) / 4;
    ledArray[centre_Pos_Cylon_Eye].b = pulse_time - 1512;
    ledArray[centre_Pos_Cylon_Eye + 1].b = (pulse_time - 1512) / 4;
    ledArray[centre_Pos_Cylon_Eye + 2].b = (pulse_time - 1512) / 8;
    
    // Set the Floodlight Diode Pin to "OFF":
    digitalWrite(PIN_FLOODLIGHT_SWITCH, LOW);
  }
  if (pulse_time >= 1768)
  {
    // Adjust pulse_time of greater than 2000:
    if (pulse_time > 2000)
    {
      pulse_time = 2000;
    }
    for (int i = 0; i < NUM_leds; i++)
    {
      val0 = (pulse_time - 1746);
      val4 = val0/4;
      val8 = val0/8;
      ledArray[centre_Pos_Cylon_Eye - 2] = CRGB(val8, val8, val8);
      ledArray[centre_Pos_Cylon_Eye - 1] = CRGB(val4, val4, val4);
      ledArray[centre_Pos_Cylon_Eye]     = CRGB(255, 255, 255);
      ledArray[centre_Pos_Cylon_Eye + 1] = CRGB(val4, val4, val4);
      ledArray[centre_Pos_Cylon_Eye + 2] = CRGB(val8, val8, val8);
    }
    // Set the Floodlight Diode Pin to "OFF":
    digitalWrite(PIN_FLOODLIGHT_SWITCH, LOW);
  }

  // UPDATE THE LCD DISPLAY:
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);
}



