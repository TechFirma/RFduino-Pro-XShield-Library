/*************************************************************
  Library for the RFduino Pro XShield Port Expander
  Tech Firma, LLC - info@techfirma.com

  -----------------------------
  Update History:
  Rev 1.0  08/31/2015  Kurt Schulz
  : Initial version derived from Adafruit MCP23017 library

  -----------------------------

  Derived from Adafruit_MCP23017 library:
  	These displays use I2C to communicate. 2 pins are required to
  	interface
  	Adafruit invests time and resources providing this open source code,
  	please support Adafruit and open-source hardware by purchasing
  	products from Adafruit!

  	Written by Limor Fried/Ladyada for Adafruit Industries.
  	BSD license, all text above must be included in any redistribution
 *************************************************************/

#include <Wire.h>
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif
#include "RFduinoProXShield.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// minihelper to keep Arduino backward compatibility
static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
	Wire.write((uint8_t) x);
#else
	Wire.send(x);
#endif
}

static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
	return Wire.read();
#else
	return Wire.receive();
#endif
}

/**
 * Bit number associated to a give Pin
 */
uint8_t RFduinoProXShield::bitForPin(uint8_t pin){
	return pin%8;
}

/**
 * Register address, port dependent, for a given PIN
 */
uint8_t RFduinoProXShield::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin<8) ?portAaddr:portBaddr;
}

/**
 * Reads a given register
 */
uint8_t RFduinoProXShield::readRegister(uint8_t addr){
	// read the current GPINTEN
	Wire.beginTransmission(MCP23017_ADDRESS | i2caddr);
	wiresend(addr);
	Wire.endTransmission();
	Wire.requestFrom(MCP23017_ADDRESS | i2caddr, 1);
	return wirerecv();
}


/**
 * Writes a given register
 */
void RFduinoProXShield::writeRegister(uint8_t regAddr, uint8_t regValue){
	// Write the register
	Wire.beginTransmission(MCP23017_ADDRESS | i2caddr);
	wiresend(regAddr);
	wiresend(regValue);
	Wire.endTransmission();
}


/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void RFduinoProXShield::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
	uint8_t bit=bitForPin(pin);
	regValue = readRegister(regAddr);

	// set the value for the particular bit
	bitWrite(regValue,bit,pValue);

	writeRegister(regAddr,regValue);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Tech Firma, LLC - RFduinoProXShield
 * Initializes the MCP23017 given its I2C address and analog input enable/disable args
 */
void RFduinoProXShield::begin(uint8_t addr, uint8_t analog) {
	if (addr > 7) {
		addr = 7;
	}
	i2caddr = addr;

	Wire.begin();

	// set defaults
	if (analog != true) {
		// all inputs on port A
		writeRegister(MCP23017_IODIRA,0xff);
	} else {
		// first 4 are outputs for analog channel address, the rest are inputs on port A
		writeRegister(MCP23017_IODIRA,0xf0);
	}

	// all inputs on port B
	writeRegister(MCP23017_IODIRB,0xff);
}

/**
 * Tech Firma, LLC - RFduinoProXShield
 * Initializes the MCP23017 given its I2C address and enables analog inputs; see datasheet for address selection.
 */
void RFduinoProXShield::begin(uint8_t addr) {
	begin(addr, true);
}

/**
 * Tech Firma, LLC - RFduinoProXShield
 * Initializes the default MCP23017, with 000 for the configurable part of the I2C address and enables analog inputs
 */
void RFduinoProXShield::begin(void) {
	begin(0, true);
}

/**
 *   Tech Firma, LLC - RFduinoProXShield
 *   Sets all pins on ports A&B to INPUT or OUTPUT
 */
void RFduinoProXShield::portABMode(uint8_t d){
	uint8_t portData=(d==INPUT) ? 0xFF : 0x0;
	writeRegister(MCP23017_IODIRA,portData);
	writeRegister(MCP23017_IODIRB,portData);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void RFduinoProXShield::pinMode(uint8_t p, uint8_t d) {
	updateRegisterBit(p,(d==INPUT),MCP23017_IODIRA,MCP23017_IODIRB);
}

/**
 *   Tech Firma, LLC - RFduinoProXShield
 *   Sets up the MUX channel addressing on GPIOA port, bits D0 - D3.
 *   Returns the analog value read on GPIO pin
 */

uint16_t RFduinoProXShield::analogRead(uint8_t pin){
	if (pin > 15) {
	  pin = 15;
	}
	uint8_t gpio;

	// read the GPIOA output latches and reset the lower 4 MUX address bits
	gpio = readRegister(MCP23017_OLATA) & 0xF0;

	// write the new GPIO value with the MUX address bits OR'd in
	writeRegister(MCP23017_GPIOA,gpio|pin);

  	return ::analogRead(GPIO_MUX_COMMON);
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t RFduinoProXShield::readGPIOAB() {
	uint16_t ba = 0;
	uint8_t a;

	// read the current GPIO output latches
	Wire.beginTransmission(MCP23017_ADDRESS | i2caddr);
	wiresend(MCP23017_GPIOA);
	Wire.endTransmission();

	Wire.requestFrom(MCP23017_ADDRESS | i2caddr, 2);
	a = wirerecv();
	ba = wirerecv();
	ba <<= 8;
	ba |= a;

	return ba;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t RFduinoProXShield::readGPIO(uint8_t b) {

	// read the current GPIO output latches
	Wire.beginTransmission(MCP23017_ADDRESS | i2caddr);
	if (b == 0)
		wiresend(MCP23017_GPIOA);
	else {
		wiresend(MCP23017_GPIOB);
	}
	Wire.endTransmission();

	Wire.requestFrom(MCP23017_ADDRESS | i2caddr, 1);
	return wirerecv();
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void RFduinoProXShield::writeGPIOAB(uint16_t ba) {
	Wire.beginTransmission(MCP23017_ADDRESS | i2caddr);
	wiresend(MCP23017_GPIOA);
	wiresend(ba & 0xFF);
	wiresend(ba >> 8);
	Wire.endTransmission();
}

void RFduinoProXShield::digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit=bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr=regForPin(pin,MCP23017_OLATA,MCP23017_OLATB);
	gpio = readRegister(regAddr);

	// set the pin and direction
	bitWrite(gpio,bit,d);

	// write the new GPIO
	regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	writeRegister(regAddr,gpio);
}

void RFduinoProXShield::pullUp(uint8_t p, uint8_t d) {
	updateRegisterBit(p,d,MCP23017_GPPUA,MCP23017_GPPUB);
}

uint8_t RFduinoProXShield::digitalRead(uint8_t pin) {
	uint8_t bit=bitForPin(pin);
	uint8_t regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,flase, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void RFduinoProXShield::setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity){
	// configure the port A
	uint8_t ioconfValue=readRegister(MCP23017_IOCONA);
	bitWrite(ioconfValue,6,mirroring);
	bitWrite(ioconfValue,2,openDrain);
	bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONA,ioconfValue);

	// Configure the port B
	ioconfValue=readRegister(MCP23017_IOCONB);
	bitWrite(ioconfValue,6,mirroring);
	bitWrite(ioconfValue,2,openDrain);
	bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONB,ioconfValue);
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 *
 */
void RFduinoProXShield::setupInterruptPin(uint8_t pin, uint8_t mode) {

	// set the pin interrupt control (0 means change, 1 means compare against given value);
	updateRegisterBit(pin,(mode!=CHANGE),MCP23017_INTCONA,MCP23017_INTCONB);
	// if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

	// In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
	// In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
	updateRegisterBit(pin,(mode==FALLING),MCP23017_DEFVALA,MCP23017_DEFVALB);

	// enable the pin for interrupt
	updateRegisterBit(pin,HIGH,MCP23017_GPINTENA,MCP23017_GPINTENB);

}

uint8_t RFduinoProXShield::getLastInterruptPin(){
	uint8_t intf;

	// try port A
	intf=readRegister(MCP23017_INTFA);
	for(int i=0;i<8;i++) if (bitRead(intf,i)) return i;

	// try port B
	intf=readRegister(MCP23017_INTFB);
	for(int i=0;i<8;i++) if (bitRead(intf,i)) return i+8;

	return MCP23017_INT_ERR;

}

uint8_t RFduinoProXShield::getLastInterruptPinValue(){
	uint8_t intPin=getLastInterruptPin();
	if(intPin!=MCP23017_INT_ERR){
		uint8_t intcapreg=regForPin(intPin,MCP23017_INTCAPA,MCP23017_INTCAPB);
		uint8_t bit=bitForPin(intPin);
		return (readRegister(intcapreg)>>bit) & (0x01);
	}

	return MCP23017_INT_ERR;
}
