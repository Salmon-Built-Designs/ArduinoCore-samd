/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "OTA.h"

__attribute__ ((section(".ota_boot")))
unsigned char otaBoot[0x4000] = {
#if defined(ARDUINO_SAMD_ZERO)
  #include "boot/zero.h"
#elif defined(ARDUINO_SAMD_MKR1000)
  #include "boot/mkr1000.h"
#elif defined(ARDUINO_SAMD_MKRZERO)
  #include "boot/mkr1000.h"
#else
  #error "Unsupported board!"
#endif
};

OTAClass::OTAClass() {
}

void OTAClass::begin() {
}

void OTAClass::poll() {
}

void OTAClass::end() {
}

OTAClass OTA;
