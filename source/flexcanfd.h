/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FLEXCANFD_H_
#define _FLEXCANFD_H_

extern volatile uint32_t g_systickCounter;

#define SysTick_GetTicks() 	g_systickCounter
void SysTick_SetTicks(uint32_t n);
void SysTick_config( uint32_t configStick );
void MCXN_flexcan_init(void);
void MCXN_flexcan_Test(void);

#endif 
