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

#ifndef __XO__
#define __XO__

#include <stdint.h>

typedef struct XOPacket
{
	uint8_t type;
	uint8_t bpm;
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t slowBeats;
	uint8_t fastBeats;

} XOPacket;

int XOPacketParse(XOPacket* packetOut, uint8_t* data, uint16_t length);

#endif //__XO__
