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
 * currently it provides
 * (1) retraction/extention of the Z wheel
 * (2) control of 74CH595
 */
 
#ifndef RocombFirmata_h
#define RocombFirmata_h

#include <ConfigurableFirmata.h>
#include "FirmataFeature.h"
//#include "utility/serialUtils.h"

// Rocomb command bytes
#define ROCOMB_CONFIG               0x10
#define ROCOMB_Z_RETRACT            0x20
#define ROCOMB_Z_EXTEND             0x30
#define ROCOMB_EN_STEPPER_Z         0x40
#define ROCOMB_EN_STEPPER_LR        0x50
#define ROCOMB_EN_LCD               0x60
#define ROCOMB_EN_MPOWER            0x70
#define ROCOMB_EN_SPOWER            0x80
#define ROCOMB_TEST                 0x90 //flush pin 13 LED for 3 times

#define ROCOMB_MODE_MASK            0xF0


class RocombFirmata: public FirmataFeature
{
  public:
    boolean handlePinMode(byte pin, int mode);
    void handleCapability(byte pin);
    boolean handleSysex(byte command, byte argc, byte *argv);
    void update();
    void reset();
  private:

	int LED_count = 0;
	int TEST_count = 0;

	boolean TEST_update();
};

#endif
