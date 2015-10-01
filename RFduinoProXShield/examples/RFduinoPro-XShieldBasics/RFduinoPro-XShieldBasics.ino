/*******************************************************************************
  RFduino Pro XShield
  Example Sketch - Basics

  Copyright (C) 2015 - Tech Firma, LLC
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Tech Firma, LLC
  Cincinnati, OH  45242

  info@techfirma.com

  ****************************
  Update History:
  Rev 1.0  08/31/2015  Kurt Schulz
  : Initial version, Arduino IDE 1.60

  **************************** 
                                        LED   150ohm
  LED on XShield D4                D4--->|----/\/\/---GND (~8mA)
  
                                            _|_
  Momentary button on XShield D5   D5-------O O-------GND
  
                                                   A0
                                            LDR     |    10k
  LDR on XShield A0                3.3v---/\~/~\/---'---/\/\/---GND
  
  o Initialize the Rfduino Pro XShield port expander
  o Read the button state and light the LED when the button is pressed.
  o Read the LDR value using an analog input and send the value to the serial monitor
  
  NOTE: This example does not use Bluetooth. See the RFduino Pro example sketch 
        for using the Bluetooth features.
  
**********************************************************************************/
#include <Wire.h>
#include <RFduinoProXShield.h>

#define DEBUG                                    // Uncomment to enable debug messages, sent to serial port; set DEBUG_LEVEL to desired setting
#define DEBUG_LEVEL_LOW       1                  // Minimum number of debug log messages sent out
#define DEBUG_LEVEL_MEDIUM    2
#define DEBUG_LEVEL_HIGH      3                  // Maximum number of debug log messages sent out
#define DEBUG_LEVEL           DEBUG_LEVEL_HIGH   // Set to desired level of debug logging

#define SERIAL_BAUD           9600               // Serial for debug only

// RFD22301 Pin Assignments
#define GPIO_SERIAL_RX        0                  // Digital input pin - Serial Receive Data (input to RFD22301)
#define GPIO_SERIAL_TX        1                  // Digital output pin - Serial Transmit Data (output from RFD22301)
#define GPIO_UNUSED           2                  // Unused GPIO pin
#define GPIO_XSHIELD_INT      3                  // Digital input pin - XShield port expander interrupt (optional)
#define GPIO_MUX_COMMON       4                  // Analog input pin - XShield multiplexer common in/out (optional)
#define GPIO_I2C_SCL          5                  // Digital output pin - XShield port Expanders I2C Serial Clock
#define GPIO_I2C_SDA          6                  // Digital input/outpu pin - XShield port Expanders I2C Serial Data

// XShield I/O pins
// Remember: D0 - D3 are reserved for MUX addressing if analog inputs are enabled, leaving D4 - D15 for digital I/O.
#define D4                    4                  // XShield D4 = digital I/O 4
#define D5                    5                  // XShield D5 = digital I/O 5
#define A0                    0                  // XShield A0 = analog channel 0

RFduinoProXShield XShield;                       // Create XShield object

void setup( )
{
  #ifdef DEBUG
  Serial.begin( SERIAL_BAUD );
  delay( 50 );
  DebugAnnounceEnabled( );
  #endif

  DebugLog( DEBUG_LEVEL_HIGH, "Begin Setup... " );
  
  // Set up pins
  pinMode( GPIO_XSHIELD_INT, INPUT );
  pinMode( GPIO_MUX_COMMON, INPUT );
  
  // Initialize XShield using default I2C address 0x20 with analog inputs enabled
  // To specify different address and/or disable analog inputs, use: 
  //   begin( 1 ); -OR-  begin( 1, false );
  // Address args are 0 - 7 which equate to I2C addresses 0x20 - 0x27.
  // Pass true to enable or false to disable analog inputs
  XShield.begin( );
  
  XShield.portABMode( OUTPUT );                  // All I/O to output so we don't have pesky floating unused inputs
  XShield.pinMode( D5, INPUT );                  // Set pin D5 as input (button)
  XShield.pullUp( D5, HIGH );                    // Enable D5 100K internal pull-up resistor
  
  DebugLog( DEBUG_LEVEL_HIGH, "Pin modes configured... " );
  
  DebugLog( DEBUG_LEVEL_HIGH, "End Setup... " );
}

void loop( )
{
  byte val = XShield.digitalRead( D5 );           // Read button state on D5
  XShield.digitalWrite( D4, !val );               // Indicate button state using LED on D4

  int analogVal = XShield.analogRead( A0 );       // Read LDR from A0 and send value to serial monitor
  DebugLog( DEBUG_LEVEL_MEDIUM, "Analog Input: ", analogVal );  
}

// ------------------------------------------------
// Debug Log Functions
// Send debug messages to the serial port if DEBUG enabled
// ------------------------------------------------

void DebugAnnounceEnabled( )
{
  switch( DEBUG_LEVEL )
  {
    case DEBUG_LEVEL_LOW:
    Serial.println( "\nDEBUG_LEVEL_LOW enabled..." );
    break;
    
    case DEBUG_LEVEL_MEDIUM:
    Serial.println( "\nDEBUG_LEVEL_MEDIUM enabled..." );
    break;
    
    case DEBUG_LEVEL_HIGH:
    Serial.println( "\nDEBUG_LEVEL_HIGH enabled..." );
    break;
  }    
}

void DebugLog( int Level, char *Label )
{
  #ifdef DEBUG
  if( Level <= DEBUG_LEVEL )
  {
    Serial.println( Label );
  }
  #endif
}

void DebugLog( int Level, char *Label, int Var )
{
  #ifdef DEBUG
  if( Level <= DEBUG_LEVEL )
  {
    Serial.print( Label );
    Serial.println( Var );
  }
  #endif
}

void DebugLog( int Level, char *Label, int Var1, int Var2 )
{
  #ifdef DEBUG
  if( Level <= DEBUG_LEVEL )
  {
    Serial.print( Label );
    Serial.print( Var1 );
    Serial.print( ", " );
    Serial.println( Var2 );
  }
  #endif
}

