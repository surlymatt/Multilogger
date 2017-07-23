/*
 K-type temperature logger:
 Matt Swan 2017
 
 This code contains content from:
    http://www.freetronics.com/products/lcd-keypad-shield
    RTIMULib-Arduino, Copyright (c) 2014-2015, richards-tech
    TCMux shield demo by Ocean Controls
  ---------------------------------------------------------------------
  
  LCD control on the Freetronics LCD & Keypad Shield:
  ADC voltages for the 5 buttons on analog input pin A0:
    RIGHT:  0.00V :   0 @ 8bit ;   0 @ 10 bit
    UP:     0.71V :  36 @ 8bit ; 145 @ 10 bit
    DOWN:   1.61V :  82 @ 8bit ; 329 @ 10 bit
    LEFT:   2.47V : 126 @ 8bit ; 505 @ 10 bit
    SELECT: 3.62V : 185 @ 8bit ; 741 @ 10 bit
*/
/*--------------------------------------------------------------------------------------
  Includes
--------------------------------------------------------------------------------------*/
#include <Wire.h>
#include <LiquidCrystal.h>   // include LCD library
#include <SD.h>              // SD card library
#include "I2Cdev.h"          // I2C support for 9 DOF IMU
#include "RTIMUSettings.h"   // 9 DOF IMU
#include "RTIMU.h"           // 9 DOF IMU
#include "RTFusionRTQF.h"    // 9 DOF IMU calculation engine for resolving vectors
#include "CalLib.h"          // 9 DOF IMU Calibration tools
#include <EEPROM.h>
#include <string.h>          // Use the string Library
#include <ctype.h>
/*--------------------------------------------------------------------------------------
  Defines
--------------------------------------------------------------------------------------*/
// LCD IO pins in use
#define BUTTON_ADC_PIN           A0  // A0 is the button ADC input
#define LCD_BACKLIGHT_PIN         3  // D3 controls LCD backlight
#define LCD_RS                    1 // Register Select, was 8
#define LCD_E                     0 // Enable, was 9
#define LCD_D4                    4 // was 4
#define LCD_D5                    5 // was 5
#define LCD_D6                    6 // was 6
#define LCD_D7                    2 // was 7
#define uSD_CS                    8 // SD card Chip Select
// T
// ADC readings expected for the 5 buttons on the ADC input
#define RIGHT_10BIT_ADC           0  // right
#define UP_10BIT_ADC            145  // up
#define DOWN_10BIT_ADC          329  // down
#define LEFT_10BIT_ADC          505  // left
#define SELECT_10BIT_ADC        741  // select
#define BUTTONHYSTERESIS         10  // hysteresis for valid button sensing window
//return values for ReadButtons()
#define BUTTON_NONE               0  // 
#define BUTTON_RIGHT              1  // 
#define BUTTON_UP                 2  // 
#define BUTTON_DOWN               3  // 
#define BUTTON_LEFT               4  // 
#define BUTTON_SELECT             5  // 
//some example macros with friendly labels for LCD backlight/pin control, tested and can be swapped into the example code as you like
#define LCD_BACKLIGHT_OFF()     digitalWrite( LCD_BACKLIGHT_PIN, LOW )
#define LCD_BACKLIGHT_ON()      digitalWrite( LCD_BACKLIGHT_PIN, HIGH )
#define LCD_BACKLIGHT(state)    { if( state ){digitalWrite( LCD_BACKLIGHT_PIN, HIGH );}else{digitalWrite( LCD_BACKLIGHT_PIN, LOW );} }
#define LCB_BACKLIGHT_RATE_MS    30  // Flash rate (ms)
#define  SERIAL_PORT_SPEED  115200

/*--------------------------------------------------------------------------------------
  Variables and objects
--------------------------------------------------------------------------------------*/
byte buttonJustPressed  = false;         //this will be true after a ReadButtons() call if triggered
byte buttonJustReleased = false;         //this will be true after a ReadButtons() call if triggered
byte buttonWas          = BUTTON_NONE;   //used by ReadButtons() for detection of button events

// IMU objects
RTIMU *imu;                              // the IMU object
RTFusionRTQF fusion;                     // the fusion object
RTIMUSettings settings;                  // the settings object
/*--------------------------------------------------------------------------------------
  Init the LCD library with the LCD pins to be used
--------------------------------------------------------------------------------------*/
LiquidCrystal lcd( LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7 );   //Pins for the freetronics 16x2 LCD shield. LCD: ( RS, E, LCD-D4, LCD-D5, LCD-D6, LCD-D7 )

/*--------------------------------------------------------------------------------------
  setup()
  Called by the Arduino framework once, before the main loop begins
--------------------------------------------------------------------------------------*/
void setup()
{
   int errcode;

   // LCD setup
   // button adc input
   pinMode( BUTTON_ADC_PIN, INPUT );         //ensure A0 is an input
   digitalWrite( BUTTON_ADC_PIN, LOW );      //ensure pullup is off on A0
   //lcd backlight control
   digitalWrite( LCD_BACKLIGHT_PIN, HIGH );  //backlight control pin D3 is high (on)
   pinMode( LCD_BACKLIGHT_PIN, OUTPUT );     //D3 is an output
   
   // LCD screen initialisation
   //set up the LCD number of columns and rows: 
   lcd.begin( 16, 2 );
   //Print some initial text to the LCD.
   lcd.setCursor( 0, 0 );   //top left
   //          1234567890123456
   lcd.print( "K-type TC logger" );
   //
   lcd.setCursor( 0, 1 );   //bottom left
   //          1234567890123456
   lcd.print( "Btn:" );

  // Open serial communications and wait for port to open:
  Serial.begin(SERIAL_PORT_SPEED);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // 9DOF IMU setup
    Wire.begin();
    imu = RTIMU::createIMU(&settings);      // create the imu object
  
    Serial.print("IMU on "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("IMU init Fail: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Compass cal ok.");
    else
        Serial.println("No compass cal.");

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used. In-between gives the fusion mix.
    fusion.setSlerpPower(0.02);
    
    // Use of sensors in the fusion algorithm can be controlled here.  Change any of these to false to disable that sensor
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);


  // uSD card shield initialisation
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to  output, even if you don't use it:
  pinMode(uSD_CS, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(uSD_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}
/*--------------------------------------------------------------------------------------
  loop()
  Arduino main loop
--------------------------------------------------------------------------------------*/
void loop()
{
   byte button;
   byte timestamp;

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // this opens the file and appends to the end of file
  // if the file does not exist, this will create a new file.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // uSD card demo code pasted in!
  // make a string for assembling the data to log:
  String dataString = "";

    //get the latest button pressed, also the buttonJustPressed, buttonJustReleased flags
   button = ReadButtons();
   //blank the demo text line if a new button is pressed or released, ready for a new label to be written
   if( buttonJustPressed || buttonJustReleased )
   {
     lcd.setCursor( 4, 1 );
     lcd.print( "            " );
   }
   //show text label for the button pressed
   switch( button )
   {
      case BUTTON_NONE:
      {
         break;
      }
      case BUTTON_RIGHT:
      {
         lcd.setCursor( 4, 1 );
         lcd.print( "RIGHT" );
         break;
      }
      case BUTTON_UP:
      {
         lcd.setCursor( 4, 1 );
         lcd.print( "UP" );
         break;
      }
      case BUTTON_DOWN:
      {
         lcd.setCursor( 4, 1 );
         lcd.print( "DOWN" );
         break;
      }
      case BUTTON_LEFT:
      {
        lcd.setCursor( 4, 1 );
        lcd.print( "LEFT" );
        break;
     }
     case BUTTON_SELECT:
     {
        lcd.setCursor( 4, 1 );
        lcd.print( "SELECT-FLASH" );    
        
        // an example of LCD backlight control via a macro with nice label, called with a value
        LCD_BACKLIGHT(false);
        delay( LCB_BACKLIGHT_RATE_MS );
        LCD_BACKLIGHT(true);   //leave the backlight on at exit
        delay( LCB_BACKLIGHT_RATE_MS );
        
        break;
      }
      default:
     {
        break;
     }
   }
   // print the number of seconds since reset (two digits only)
   timestamp = ( (millis() / 1000) % 100 );   //"% 100" is the remainder of a divide-by-100, which keeps the value as 0-99 even as the result goes over 100
   lcd.setCursor( 14, 1 );
   if( timestamp <= 9 )
      lcd.print( " " );   //quick trick to right-justify this 2 digit value when it's a single digit
   lcd.print( timestamp, DEC );

   //clear the buttonJustPressed or buttonJustReleased flags, they've already done their job now.
   if( buttonJustPressed )
      buttonJustPressed = false;
   if( buttonJustReleased )
      buttonJustReleased = false;



  // if the file is available, write to it:
  if (dataFile)   {
    int timeStamp = millis();
    //write to uSD card
    dataFile.print(timeStamp);
    dataFile.println(); //create a new row to read data more clearly
  }  
  // if the file isn't open, pop up an error:
  else
  {
    Serial.println("error opening datalog.txt");
  } 
}
/*--------------------------------------------------------------------------------------
  ReadButtons()
  Detect the button pressed and return the value
  Uses global values buttonWas, buttonJustPressed, buttonJustReleased.
--------------------------------------------------------------------------------------*/
byte ReadButtons()
{
   unsigned int buttonVoltage;
   byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn
   
   //read the button ADC pin voltage
   buttonVoltage = analogRead( BUTTON_ADC_PIN );
   //sense if the voltage falls within valid voltage windows
   if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
   }
   //handle button flags for just pressed and just released events
   if( ( buttonWas == BUTTON_NONE ) && ( button != BUTTON_NONE ) )
   {
      //the button was just pressed, set buttonJustPressed, this can optionally be used to trigger a once-off action for a button press event
      //it's the duty of the receiver to clear these flags if it wants to detect a new button change event
      buttonJustPressed  = true;
      buttonJustReleased = false;
   }
   if( ( buttonWas != BUTTON_NONE ) && ( button == BUTTON_NONE ) )
   {
      buttonJustPressed  = false;
      buttonJustReleased = true;
   }
   
   //save the latest button value, for change event detection next time round
   buttonWas = button;
   
   return( button );
}
