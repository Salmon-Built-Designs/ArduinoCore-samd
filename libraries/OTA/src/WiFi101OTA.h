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

#ifndef _WIFI101_OTA_H_INCLUDED
#define _WIFI101_OTA_H_INCLUDED

#include "WiFi101.h"

class WiFiOTAClass {
public:
  WiFiOTAClass();

  void begin();
  void poll();

private:
  void sendHttpResponse(Client& client, int code, const char* status);

private:
  WiFiServer _server;
};

extern WiFiOTAClass WiFiOTA;

#endif
