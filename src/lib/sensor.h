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


#ifndef EPOS_SENSOR_H
#define EPOS_SENSOR_H

#include "device.h"

/** \file sensor.h
  * \brief EPOS sensor functions
  */

/** Predefined EPOS sensor constants
  */
#define EPOS_SENSOR_INDEX_CONFIGURATION         0x2210
#define EPOS_SENSOR_SUBINDEX_PULSES             0x01
#define EPOS_SENSOR_SUBINDEX_TYPE               0x02
#define EPOS_SENSOR_SUBINDEX_POLARITY           0x04
#define EPOS_SENSOR_INDEX_POSITION              0x2020

/** Predefined EPOS sensor error codes
  */
#define EPOS_SENSOR_ERROR_NONE                  0
#define EPOS_SENSOR_ERROR_SETUP                 1

/** \brief Predefined EPOS sensor error descriptions
  */
extern const char* epos_sensor_errors[];

/** \brief EPOS position sensor types
  */
typedef enum {
  epos_sensor_3chan = 1,
  epos_sensor_2chan = 2,
  epos_sensor_hall = 3
} epos_sensor_type_t;

/** \brief EPOS position sensor polarity
  */
typedef enum {
  epos_sensor_normal = 0x00,
  epos_sensor_inverted = 0x03
} epos_sensor_polarity_t;

typedef enum {
  epos_sensor_fully_supervised = 0x00,
  epos_sensor_hardware_supervised = 0x01,
  epos_sensor_software_supervised = 0x02,
  epos_sensor_unsupervised = 0x03
} epos_sensor_supervision_t;

/** \brief Structure defining an EPOS position sensor
  */
typedef struct epos_sensor_t {
  epos_device_p dev;               //!< The EPOS device of the sensor.

  epos_sensor_type_t type;         //!< The position sensor type.
  epos_sensor_polarity_t polarity; //!< The position sensor polarity.
  int num_pulses;                  //!< The number of pulses per revolution.

  epos_sensor_supervision_t
    supervision;                   //!< The position sensor's supervision.
} epos_sensor_t, *epos_sensor_p;

/** \brief Initialize EPOS position sensor
  * \param[in] sensor The EPOS position sensor to be initialized.
  * \param[in] dev The EPOS device the position sensor is connected to.
  * \param[in] type The type of the EPOS position sensor to be initialized.
  * \param[in] polarity The polarity of the position sensor.
  * \param[in] num_pulses The sensor's number of pulses per revolution.
  * \param[in] supervision The sensor's supervision.
  */
void epos_sensor_init(
  epos_sensor_p sensor,
  epos_device_p dev,
  epos_sensor_type_t type,
  epos_sensor_polarity_t polarity,
  int num_pulses,
  epos_sensor_supervision_t supervision);

/** \brief Destroy EPOS position sensor
  * \param[in] sensor The EPOS position sensor to be destroyed.
  */
void epos_sensor_destroy(
  epos_sensor_p sensor);

/** \brief Set EPOS sensor parameters
  * \param[in] sensor The EPOS sensor to set the parameters for.
  * \return The resulting error code.
  */
int epos_sensor_setup(
  epos_sensor_p sensor);

/** \brief Retrieve EPOS position sensor type
  * \param[in] sensor The EPOS position sensor to retrieve the type for.
  * \return The type of the specified EPOS position sensor.
  */
epos_sensor_type_t epos_sensor_get_type(
  epos_sensor_p sensor);

/** \brief Set EPOS position sensor type
  * \param[in] sensor The EPOS position sensor to set the type for.
  * \param[in] type The type of the specified EPOS position sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_type(
  epos_sensor_p sensor,
  epos_sensor_type_t type);

/** \brief Retrieve EPOS position sensor polarity
  * \param[in] sensor The EPOS position sensor to retrieve the polarity for.
  * \return The polarity of the specified EPOS position sensor.
  */
epos_sensor_polarity_t epos_sensor_get_polarity(
  epos_sensor_p sensor);

/** \brief Set EPOS position sensor polarity
  * \param[in] sensor The EPOS position sensor to set the polarity for.
  * \param[in] polarity The polarity of the specified EPOS position sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_polarity(
  epos_sensor_p sensor,
  epos_sensor_polarity_t polarity);

/** \brief Retrieve an EPOS position sensor's number of pulses per revolution
  * \param[in] sensor The EPOS position sensor to retrieve the number of
  *   revolutions for.
  * \return The number of pulses of the specified EPOS position sensor.
  */
int epos_sensor_get_pulses(
  epos_sensor_p sensor);

/** \brief Set an EPOS position sensor's number of pulses per revolution
  * \param[in] sensor The EPOS position sensor to set the number of
  *   revolutions for.
  * \param[in] num_pulses The number of pulses per revolution of the specified
  *   EPOS position sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_pulses(
  epos_sensor_p sensor,
  int num_pulses);

/** \brief Retrieve an EPOS position sensor's supervision
  * \param[in] sensor The EPOS position sensor to retrieve the supervision for.
  * \return The number of pulses of the specified EPOS position sensor.
  */
epos_sensor_supervision_t epos_sensor_get_supervision(
  epos_sensor_p sensor);

/** \brief Set an EPOS position sensor's supervision
  * \param[in] sensor The EPOS position sensor to set the supervision for.
  * \param[in] supervision The supervision of the specified EPOS position
*     sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_supervision(
  epos_sensor_p sensor,
  epos_sensor_supervision_t supervision);

/** \brief Retrieve an EPOS position sensor's position
  * \param[in] sensor The EPOS position sensor to retrieve the position for.
  * \return The position of the specified EPOS position sensor in [steps].
  */
short epos_sensor_get_position(
  epos_sensor_p sensor);

#endif
