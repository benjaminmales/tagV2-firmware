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

#include <math.h>

#include "ascii-man.h"

//@@ debug
#include <stdio.h>

int AsciiArrayToUint8(uint8_t* byteArray, int length, uint8_t* out)
{
  int index = 1;

  // boundary checks.
  if(length > 3)
    return(-1);

  (*out) = 0;
  while((length - index) >= 0)
  {
  	(*out) += ((byteArray[length - index] - 0x30) * pow(10, (index-1)));
    index++;
  }

  return(0);
}