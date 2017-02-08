// Copyright 2016 Highway1
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <stdio.h>

#include "XO.h"

#include "ascii-man.h"

int XOPacketParse(XOPacket* packetOut, uint8_t* data, uint16_t length)
{
  uint8_t index = 2;

  const int newStringMaxLength = 4;
  int       newStringLength = 0;
  uint8_t   newString[newStringLength];

  uint8_t packetValues[7] = {0x00};
  int     packetValuesIndex = 0;

  // check packet size for validity
  if(length < 13 || length > 30) //@@ 30 is just a guess
    return(6); // invalid packet size

  // ensure valid pointers
  if(data == 0 || packetOut == 0)
    return(5); // null pointer

  // find first comma (info before first comma is packet type.)
  for(int i = 0; i < length; i++)
  {
    if(data[i] == ',')
    {
      index = i + 1;
      break;
    }
  }

  if(index == 0)
    return(4); // invalid format. (no ',' found)

  index = 0;

  while(index < length)
  {
    if(data[index] == ',')
    {
      AsciiArrayToUint8(newString, newStringLength, &packetValues[packetValuesIndex++]);
      memset(newString, 0x00, newStringMaxLength);
      newStringLength = 0;
    }
    else
    {
      newString[newStringLength++] = data[index];

      if(newStringLength == newStringMaxLength)
        index = ~0;
    }

    index++;
  }

  int i = 0;
  packetOut->type       = packetValues[i++];
  packetOut->bpm        = packetValues[i++];
  packetOut->red        = packetValues[i++];
  packetOut->green      = packetValues[i++];
  packetOut->blue       = packetValues[i++];
  packetOut->slowBeats  = packetValues[i++];
  packetOut->fastBeats  = packetValues[i++];

  return(0);
}
