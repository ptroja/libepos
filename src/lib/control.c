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

#include <tulibs/timer.h>

#include "control.h"

void epos_control_init(epos_control_p control, epos_device_p dev,
  epos_control_type_t type) {
  control->dev = dev;
}

void epos_control_destroy(epos_control_p control) {
  if (control->dev)
    control->dev = 0;
}

epos_control_type_t epos_control_get_type(epos_control_p control) {
  char type;
  epos_device_read(control->dev, EPOS_CONTROL_INDEX_MODE_DISPLAY, 0, &type, 1);

  return type;
}

int epos_control_set_type(epos_control_p control, epos_control_type_t type) {
  char t = type;
  int result = epos_device_write(control->dev, EPOS_CONTROL_INDEX_MODE, 0,
    &t, 1);

  if (!result)
    control->type = type;

  return result;
}

int epos_control_start(epos_control_p control) {
  int result = epos_device_set_control(control->dev,
    EPOS_DEVICE_CONTROL_ENABLE_OPERATION);

  if (!result)
    timer_sleep(EPOS_CONTROL_START_SLEEP);

  return result;
}

int epos_control_stop(epos_control_p control) {
  return epos_device_set_control(control->dev,
    EPOS_DEVICE_CONTROL_QUICK_STOP);
}
