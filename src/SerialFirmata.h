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
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2013 Jeff Hoefs.  All rights reserved.
  Copyright (C) 2013 Norbert Truchsess. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  formatted using the GNU C formatting and indenting
*/

/*
 * TODO: use Program Control to load stored profiles from EEPROM
 */
 
#ifndef SerialFirmata_h
#define SerialFirmata_h

#include <ConfigurableFirmata.h>
#include "FirmataFeature.h"
#include "utility/serialUtils.h"

class SerialFirmata: public FirmataFeature
{
  public:
    boolean handlePinMode(byte pin, int mode);
    void handleCapability(byte pin);
    boolean handleSysex(byte command, byte argc, byte *argv);
    void update();
    void reset();
  private:
    /* serial message */
	Stream *swSerial0 = NULL;
	Stream *swSerial1 = NULL;
	Stream *swSerial2 = NULL;
	Stream *swSerial3 = NULL;

	byte reportSerial[MAX_SERIAL_PORTS];
	int serialBytesToRead[12];
	signed char serialIndex = -1;
	
	Stream* getPortFromId(byte portId);
	void checkSerial();
};

#endif
