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


#ifndef EPOS_ERROR_H
#define EPOS_ERROR_H

#include "device.h"

/** \file error.h
  * \brief EPOS error functions
  */

/** Predefined EPOS error constants
  */
#define EPOS_ERROR_INDEX_HISTORY                0x1003
#define EPOS_ERROR_SUBINDEX_HISTORY_LENGTH      0x00
#define EPOS_ERROR_SUBINDEX_HISTORY_ENTRIES     0x01

/** \brief Structure defining an EPOS communication error
  */
typedef struct epos_error_comm_t {
  int code;             //!< The code of the EPOS communication error.
  const char* message;  //!< A descriptive message of the communication error.
} epos_error_comm_t, *epos_error_comm_p;

/** \brief Structure defining an EPOS device error
  */
typedef struct epos_error_device_t {
  short code;           //!< The code of the EPOS device error.
  unsigned char reg;    //!< The register value of the EPOS device error.
  const char* message;  //!< A descriptive message of the EPOS device error.
} epos_error_device_t, *epos_error_device_p;

/** \brief Predefined EPOS communication errors
  */
extern epos_error_comm_t epos_errors_comm[];

/** \brief Predefined EPOS device errors
  */
extern epos_error_device_t epos_errors_device[];

/** \brief Return an EPOS communication error message
  * \param[in] code The communication error code for which a description
  *   will be returned.
  * \return The communication error description corresponding to the
  *   specified error code.
  */
const char* epos_error_comm(
  int code);

/** \brief Return an EPOS device error message
  * \param[in] code The device error code for which a description
  *   will be returned.
  * \return The device error description corresponding to the
  *   specified error code.
  */
const char* epos_error_device(
  short code);

/** \brief Retrieve length of the EPOS device error history
  * \param[in] dev The EPOS device to retrieve the error history length for.
  * \return The length of the error history of the specified EPOS device.
  */
unsigned char epos_error_get_history_length(
  epos_device_p dev);

/** \brief Retrieve EPOS device error history
  * \param[in] dev The EPOS device to retrieve the error history for.
  * \param[out] history The error history for the specified EPOS device.
  * \return The length of the error history of the specified EPOS device.
  */
unsigned char epos_error_get_history(
  epos_device_p dev,
  epos_error_device_t history[]);

/** \brief Clear EPOS device error history
  * \param[in] dev The EPOS device to clear the error history for.
  * \return The resulting device error code.
  */
int epos_error_clear_history(
  epos_device_p dev);

#endif
