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

#include <SD.h>

#include "OTA.h"

#include "WiFi101OTA.h"

#define UPDATE_FILE "UPDATE.BIN"

WiFiOTAClass::WiFiOTAClass() :
  _server(65080)
{
}

void WiFiOTAClass::begin()
{
  _server.begin();
}

void WiFiOTAClass::poll()
{
  WiFiClient client = _server.available();

  if (client) {
    String request = client.readStringUntil('\n');
    request.trim();

    if (request != "POST /sketch HTTP/1.1") {
      sendHttpResponse(client, 404, "Not Found");
      return;
    }

    String header;
    long contentLength = -1;

    do {
      header = client.readStringUntil('\n');
      header.trim();
      Serial.println(header);

      if (header.startsWith("Content-Length: ")) {
        header.remove(0, 16);

        contentLength = header.toInt();
      }
    } while (header != "");

    if (contentLength <= 0) {
      sendHttpResponse(client, 400, "Bad Request");
      return;
    }

    File out = SD.open(UPDATE_FILE, FILE_WRITE);

    if (!out) {
      sendHttpResponse(client, 500, "Internal Server Error");
      return;
    }

    long read = 0;

    while (client.connected() && read < contentLength) {
      if (client.available()) {
        read++;

        out.write((char)client.read());
      }
    }

    out.close();

    if (read == contentLength) {
      sendHttpResponse(client, 200, "OK");

      OTA.reset();
    } else {
      SD.remove(UPDATE_FILE);

      client.stop();
    }
  }
}

void WiFiOTAClass::sendHttpResponse(Client& client, int code, const char* status)
{
  while (client.available()) {
    client.read();
  }
  
  client.print("HTTP/1.1 ");
  client.print(code);
  client.print(" ");
  client.println(status);
  client.println("Connection: close");
  client.println();
  client.stop();
}

WiFiOTAClass WiFiOTA;
