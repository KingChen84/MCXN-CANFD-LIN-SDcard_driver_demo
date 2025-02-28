/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCXN_CANFD_H_
#define _MCXN_CANFD_H_


#include "fsl_sd.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "app.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "sdmmc_config.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"
#include "flexcanfd.h"

extern FIL g_fileObject;   /* File object */
extern FIL canfd_fileObject;   /* File object */
void close_lin_file( void );
void close_canfd_file( void );
void write_lin_file( uint8_t *writedata, uint8_t len );
void write_canfd_file( uint8_t *writedata, uint8_t len );


#endif 
