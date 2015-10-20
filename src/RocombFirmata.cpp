/*
 * Firmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please clink on the following link
 * to open the download page in your default browser.
 *
 * http://firmata.org/wiki/Download
 */

/*
  Copyright (C) 2015 Eric Wang. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  formatted using the GNU C formatting and indenting
*/

/*
 * functions for the Rocomb Firmata
 * Rocomb is a virtual "device" attached to firmata
 * it accept command to trigger a set of orchistration for multiple devices attached with predefined logic
 */
 
#include <ConfigurableFirmata.h>
#include "RocombFirmata.h"

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/


// Check serial ports that have READ_CONTINUOUS mode set and relay any data
// for each port to the device attached to that port.
boolean RocombFirmata::TEST_update()
{
	if(LED_count > 0 || TEST_count >0 ) {
		if (TEST_count <= 0 && LED_count > 0) {
			TEST_count = 30000;
			LED_count--;
			if (LED_count % 2  ) digitalWrite(13,HIGH);
			else digitalWrite(13,LOW);
		}
		TEST_count--;
		if(LED_count == 0 && TEST_count == 0)return true;
	}
	return false;
}



// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
boolean RocombFirmata::handlePinMode(byte pin, int mode)
{
  //eric wang do nothing here as rocomb will not set pin to rocomb mode
  //set pin to the mode required by functions
  //if( mode == MODE_SERIAL )  {
      // used for both HW and SW serial
      //pinConfig[pin] = MODE_SERIAL;
	  //Firmata.setPinMode(pin, MODE_SERIAL); //comment out by eric for test, to avoid conflict with Firmata setting
      return true;
  //}
  //return false;
}

void RocombFirmata::handleCapability(byte pin)
{
	//eric wang do nothing here as rocomb will not set pin to rocomb mode
	//set pin to the mode required by functions
    //if (IS_PIN_SERIAL(pin)) {
    //      Firmata.write(MODE_SERIAL);
    //      Firmata.write(getSerialPinType(pin));
    //}
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/
boolean RocombFirmata::handleSysex(byte command, byte argc, byte *argv)
{
  if( command == ROCOMB_MESSAGE ) {
 
      byte mode;
	  
      mode = argv[0] & ROCOMB_MODE_MASK;
 
      switch (mode) {
        case ROCOMB_TEST:
          {
			  Firmata.setPinMode(13,OUTPUT);
			  //digitalWrite(13,HIGH);
			  LED_count  = (argv[0] & 0x0f) *2;
            break; // SERIAL_CONFIG
          }
        case ROCOMB_CONFIG:
          {
	
            break; // SERIAL_WRITE
          }
		default:
		  return false;
      }
	  //end of switch
	  return true;
  }
  return false;
}


/*==============================================================================
 * SETUP()
 *============================================================================*/

void RocombFirmata::reset()
{
  //initialize the rocomb
  
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void RocombFirmata::update()
{
  //check 
 
  if (TEST_update()) {
          Firmata.write(START_SYSEX);
          Firmata.write(ROCOMB_MESSAGE);
          Firmata.write(0x11);
          Firmata.write(END_SYSEX);
        }
}

