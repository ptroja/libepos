/*
Copyright 2008 Ralf Kaestner <ralf.kaestner@gmail.com>
ASL-ETHZ http://www.asl.ethz.ch/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <stdio.h>

#include "profile.h"

int epos_profile_wait(epos_node_p node, double timeout) {
  return epos_device_wait_status(&node->dev, EPOS_PROFILE_STATUS_REACHED,
    timeout);
}

int epos_profile_set_acceleration(epos_device_p dev, unsigned int
  acceleration) {
  return epos_device_write(dev, EPOS_PROFILE_INDEX_ACCELERATION, 0,
    (unsigned char*)&acceleration, sizeof(unsigned int));
}

int epos_profile_set_deceleration(epos_device_p dev, unsigned int
  deceleration) {
  return epos_device_write(dev, EPOS_PROFILE_INDEX_DECELERATION, 0,
    (unsigned char*)&deceleration, sizeof(unsigned int));
}

int epos_profile_set_type(epos_device_p dev, epos_profile_type_t type) {
  return epos_device_write(dev, EPOS_PROFILE_INDEX_TYPE, 0,
    (unsigned char*)&type, sizeof(short));
}
