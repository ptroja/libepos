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


#ifndef EPOS_CONTROL_H
#define EPOS_CONTROL_H

#include "device.h"

/** \file control.h
  * \brief EPOS generic control functions
  */

/** Predefined EPOS control constants
  */
#define EPOS_CONTROL_START_SLEEP             0.01

#define EPOS_CONTROL_INDEX_MODE              0x6060
#define EPOS_CONTROL_INDEX_MODE_DISPLAY      0x6061

/** \brief EPOS controller types
  */
typedef enum {
  epos_control_homing = 6,
  epos_control_profile_vel = 3,
  epos_control_profile_pos = 1,
  epos_control_position = -1,
  epos_control_velocity = -2,
  epos_control_current = -3,
  epos_control_diagnostic = -4,
  epos_control_master_enc = -5,
  epos_control_step_dir = -6
} epos_control_type_t;

/** \brief Structure defining an EPOS controller
  */
typedef struct epos_control_t {
  epos_device_p dev;          //!< The EPOS device of the controller.

  epos_control_type_t type;   //!< The controller type.
} epos_control_t, *epos_control_p;

/** \brief Initialize EPOS controller
  * \param[in] control The EPOS controller to be initialized.
  * \param[in] dev The controller's EPOS device.
  * \param[in] type The type of the EPOS controller to be initialized.
  */
void epos_control_init(
  epos_control_p control,
  epos_device_p dev,
  epos_control_type_t type);

/** \brief Destroy EPOS controller
  * \param[in] control The EPOS controller to be destroyed.
  */
void epos_control_destroy(
  epos_control_p control);

/** \brief Retrieve EPOS controller type
  * \param[in] control The EPOS controller to retrieve the type for.
  * \return The type of the specified EPOS controller.
  */
epos_control_type_t epos_control_get_type(
  epos_control_p control);

/** \brief Set EPOS controller type
  * \param[in] control The EPOS controller to set the type for.
  * \param[in] type The type of the specified EPOS controller.
  * \return The resulting device error code.
  */
int epos_control_set_type(
  epos_control_p control,
  epos_control_type_t type);

/** \brief Start EPOS controller
  * \param[in] control The EPOS controller to be started.
  * \return The resulting device error code.
  */
int epos_control_start(
  epos_control_p control);

/** \brief Stop EPOS controller
  * \param[in] control The EPOS controller to be stopped.
  * \return The resulting device error code.
  */
int epos_control_stop(
  epos_control_p control);

#endif
