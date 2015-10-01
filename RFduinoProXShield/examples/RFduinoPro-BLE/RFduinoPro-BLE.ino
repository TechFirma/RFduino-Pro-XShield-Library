/*******************************************************************************
  RFduino Pro
  Example Sketch - RFduino Bluetooth

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
  LED on GPIO2                GPIO2---->|----/\/\/----GND (~8mA)
  
                                           _|_
  Momentary button on GPIO3   GPIO3--------O O--------GND
  
                                          GPIO4
                                    LDR     |    10k
  LDR on GPIO4             3.3v---/\~/~\/---'---/\/\/---GND
    
  o Read button state and light LED when pressed if not overriden by LED override receive data
  o Send the button state event via Bluetooth
  o Receive LED override status via Bluetooth
  o While button is pressed, read the LDR value using an analog input and send the value via Bluetooth
  o Go to sleep (ULP mode) after a set time period of inactivity (default 10s); wake on button press
  o Send the sleep state out via Bluetooth
  
**********************************************************************************/
#include <RFduinoBLE.h>

#define DEBUG                                    // Uncomment to enable debug messages, sent to serial port; set DEBUG_LEVEL to desired setting
#define DEBUG_LEVEL_LOW       1                  // Minimum number of debug log messages sent out
#define DEBUG_LEVEL_MEDIUM    2
#define DEBUG_LEVEL_HIGH      3                  // Maximum number of debug log messages sent out
#define DEBUG_LEVEL           DEBUG_LEVEL_HIGH   // Set to desired level of debug logging

#define SERIAL_BAUD           9600               // Serial for debug only

// RFD22301 Pin Assignments
#define GPIO_SERIAL_RX        0                  // Digital input pin - Serial Receive Data (input to RFD22301)
#define GPIO_SERIAL_TX        1                  // Digital output pin - Serial Transmit Data (output from RFD22301)
#define GPIO_LED              2                  // Digital output pin - LED indicator
#define GPIO_BUTTON           3                  // Digital input pin - momentary pushbutton
#define GPIO_LDR              4                  // Analog input pin - Light Dependent Resistor
#define GPIO_5                5                  // Unused
#define GPIO_6                6                  // Unused
#define RFD_WAKE_PIN          GPIO_BUTTON        // Define pin for waking RFD22301

#define SLEEP                                    // Uncomment to enable ultra low power (sleep) mode
#define SLEEP_TIMEOUT        10000L              // Milliseconds of no activity before going to sleep
#define AWAKE                 0                  // RFduino is awake
#define ASLEEP                1                  // RFduino is asleep, in ultra-low-power mode

// Example Bluetooth byte code protocol for send/receive data
// Format: [bytecode][data][data][...] where bytecode uniquely identifies the data
const byte BUTTON_STATE = 0x01;                  // Byte code 1: data is button state
byte buttonState = LOW;                          // Button state send data: HIGH = not pressed, LOW = pressed

const byte LED_STATUS = 0x02;                    // Byte code 2: data holds the LED status
byte ledStatus = LOW;                            // LED status receive data: 0 = OFF, 1 = ON
byte savLedStatus = ledStatus;                   // Byte for saving current LED status

const byte LDR_VALUE = 0x03;                     // Byte code 3: data holds the value read from the LDR
unsigned int ldrValue = 0;                       // LDR value; 0 - 1023

const byte SLEEP_STATE = 0x04;                   // Byte code 4: sleep state
byte sleepState = AWAKE;                         // Byte for storing current sleep state

unsigned long awakeTime = 0L;                    // Milliseconds that we've been awake

void setup( )
{
  #ifdef DEBUG
  Serial.begin( SERIAL_BAUD );
  delay( 50 );
  DebugAnnounceEnabled( );
  #endif

  DebugLog( DEBUG_LEVEL_HIGH, "Begin Setup... " );
  
  // Set up pins
  pinMode( GPIO_LED, OUTPUT );
  pinMode( GPIO_BUTTON, INPUT_PULLUP );
    
  DebugLog( DEBUG_LEVEL_HIGH, "Pin modes configured... " );
   
  RFduino_pinWake( RFD_WAKE_PIN, LOW );           // Wake up when this pin goes LOW (button press)
  
  RFduinoBLE.deviceName = "RFduino Pro";
  RFduinoBLE.advertisementData = "RFduinoPro";
  RFduinoBLE.advertisementInterval = MILLISECONDS( 100 );
  RFduinoBLE.txPowerLevel = 0;                    // MIN -20, -16, -12, -8, -4, 0, or +4 dBm MAX

  RFduinoBLE.begin( );                            // Start the BLE stack
  delay( 500 );                                   // Give it time to start up
  
  DebugLog( DEBUG_LEVEL_HIGH, "RFDuino initialized... " );
  
  awakeTime = millis( );                          // Init awakeTime to now
  
  DebugLog( DEBUG_LEVEL_HIGH, "End Setup... " );
}

void loop( )
{
  byte val = digitalRead( GPIO_BUTTON );          // Read button state
  
  if( ledStatus == LOW )                          // If not overriden by Bluetooth receive data...
    digitalWrite( GPIO_LED, !val );               // Indicate button state using LED

  // Send the button state out via Bluetooth  
  if( buttonState != val )                        // If the button state changed...
  {
    buttonState = val;                            // Save new button state
    SendData( BUTTON_STATE, val );                // Send out the new button state
  }
  
  // Override LED status with value received via Bluetooth
  if( savLedStatus != ledStatus )                 // If new LED status was received...
  {
    savLedStatus = ledStatus;
    digitalWrite( GPIO_LED, ledStatus );          // Reflect received LED status
    awakeTime = millis( );                        // Keep awakeTime current if there's activity
  }
  
  // While the button is pressed, send out the LDR value
  if( buttonState == LOW )
  {
    unsigned int ldrValue = analogRead( GPIO_LDR );   // Read the LDR value (0 - 1023)
    SendData( LDR_VALUE, ldrValue );                  // Send it out
  }
  
  #ifdef SLEEP
  if( ( millis( ) - awakeTime ) > SLEEP_TIMEOUT )  // Go to sleep if no activity within SLEEP_TIMEOUT period
  {
    RFduino_resetPinWake( RFD_WAKE_PIN );          // Reset state of pin that caused wakeup
    GoToSleep( );
  }
  #endif
}

//************************
// Go to ultra low power mode
//
void GoToSleep( )
{
  DebugLog( DEBUG_LEVEL_HIGH, "Going to sleep... " );
  
  // Flash LED to indicate we're entering ULP mode
  FlashLED( 50, 5 );
  
  // Tell app we're going to sleep
  sleepState = ASLEEP;
  SendData( SLEEP_STATE, sleepState );
  
  RFduino_ULPDelay( INFINITE );                      // Switch to lower power mode
  // ZZZZzzzzz... sleep until next button press
  //
  
  DebugLog( DEBUG_LEVEL_HIGH, "Waking up... " );
    
  // Tell app we're awake
  sleepState = AWAKE;
  SendData( SLEEP_STATE, sleepState );
  
  // Blip LED once to indicate we're waking up
  FlashLED( 50, 1 );
  delay( 250 );
 
  digitalWrite( GPIO_LED, ledStatus );               // Set LED to current LED status value
  awakeTime = millis( );                             // Reset awakeTime to now
  
  if( RFduino_pinWoke( RFD_WAKE_PIN ) )
  {
    RFduino_resetPinWake( RFD_WAKE_PIN );            // Reset state of pin that caused wakeup
  }
}

//************************
// Flash LED
//
void FlashLED( int Delay, int Count )
{
  for( int i = 0; i < Count; i++ )
  {
    digitalWrite( GPIO_LED, HIGH );                  // Light LED
    delay( Delay );
    digitalWrite( GPIO_LED, LOW );                   // Finish with LED off
    delay( Delay );
  }  
}

// ------------------------------------------------
//
// RFduino Functions
//
// ------------------------------------------------

//*****************************
// SendData, Byte
// Format: [bytecode][data]
//
void SendData( byte ByteCode, byte Value )
{
  char dataPacket[ 2 ];
  
  dataPacket[ 0 ] = ByteCode;                      // First is the byte code
  dataPacket[ 1 ] = Value;                         // Second is the value (byte)
  RFduinoBLE.send( dataPacket, 2 );                // Send out 2 bytes

  // Keep awakeTime current if there's activity
  awakeTime = millis( );
  
  DebugLog( DEBUG_LEVEL_HIGH, "SendData (B): ", ByteCode, Value );
}

//*****************************
// SendData, Unsigned Integer
// Format: [bytecode][data high byte][data low byte]
//
void SendData( byte ByteCode, unsigned int Value )
{
  char dataPacket[ 3 ];
  
  dataPacket[ 0 ] = ByteCode;                      // First is the byte code
  dataPacket[ 1 ] = ( byte )( Value >> 8 );        // Shift high byte data to low byte, cast to byte
  dataPacket[ 2 ] = ( byte )( Value & 0xFF );      // Mask off high byte data, cast to byte
  RFduinoBLE.send( dataPacket, 3 );                // Send out 3 bytes

  // Keep awakeTime current if there's activity
  awakeTime = millis( );
  
  DebugLog( DEBUG_LEVEL_HIGH, "SendData (UI): ", ByteCode, Value );
}

//*****************************
// RFDuino Bluetooth Callbacks
//*****************************

//*****************************
// OnAdvertisement Callback
//
void RFduinoBLE_onAdvertisement( bool start )
{
  DebugLog( DEBUG_LEVEL_HIGH, "Advertisement: ", start );
}

//*****************************
// OnConnect Callback
//
void RFduinoBLE_onConnect( )
{
  delay( 300 );
  SendData( SLEEP_STATE, sleepState );          // Tell app whether we're awake or asleep

  DebugLog( DEBUG_LEVEL_HIGH, "Connected" );
}

//*****************************
// OnDisconnect Callback
//
void RFduinoBLE_onDisconnect( )
{
  DebugLog( DEBUG_LEVEL_HIGH, "Disconnected" );
}

//*****************************
// OnReceive Callback
//
void RFduinoBLE_onReceive( char *data, int len )
{
  switch( data[0] )
  {
    case LED_STATUS:
      ledStatus = data[1];
      DebugLog( DEBUG_LEVEL_HIGH, "Received LED STATUS data: ", ledStatus );
      break;
      
    default:
      DebugLog( DEBUG_LEVEL_HIGH, "Received Unknown (byte code, value): ", data[0], data[1] );
      break;
  }
}

//*****************************
// OnRSSI Callback
//
void RFduinoBLE_onRSSI( int rssi )
{
//  DebugLog( DEBUG_LEVEL_HIGH, "RSSI" );
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

