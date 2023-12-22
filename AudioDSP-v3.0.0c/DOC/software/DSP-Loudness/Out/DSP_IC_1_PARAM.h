/*
 * File:           C:\Users\marce\OneDrive\Desktop\GitHub Repos\AudioDSP-v3.0\AudioDSP-v3.0\DOC\software\DSP-Basic\Out\DSP_IC_1_PARAM.h
 *
 * Created:        Saturday, September 16, 2023 6:14:42 PM
 * Description:    DSP:IC 1 parameter RAM definitions.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright ©2023 Analog Devices, Inc. All rights reserved.
 */
#ifndef __DSP_IC_1_PARAM_H__
#define __DSP_IC_1_PARAM_H__


/* Module SafeLoadModule - SafeLoadModule*/
#define MOD_SAFELOADMODULE_COUNT                       10
#define MOD_SAFELOADMODULE_DEVICE                      "IC1"
#define MOD_SAFELOADMODULE_DATA_SAFELOAD0_ADDR         24576
#define MOD_SAFELOADMODULE_DATA_SAFELOAD1_ADDR         24577
#define MOD_SAFELOADMODULE_DATA_SAFELOAD2_ADDR         24578
#define MOD_SAFELOADMODULE_DATA_SAFELOAD3_ADDR         24579
#define MOD_SAFELOADMODULE_DATA_SAFELOAD4_ADDR         24580
#define MOD_SAFELOADMODULE_ADDRESS_SAFELOAD_ADDR       24581
#define MOD_SAFELOADMODULE_NUM_SAFELOAD_ADDR           24582

/* Module DC1 - DC Input Entry*/
#define MOD_DC1_COUNT                                  1
#define MOD_DC1_DEVICE                                 "IC1"
#define MOD_DC1_DCINPALG145X1VALUE_ADDR                20
#define MOD_DC1_DCINPALG145X1VALUE_VALUE               SIGMASTUDIOTYPE_8_24_CONVERT(1)
#define MOD_DC1_DCINPALG145X1VALUE_TYPE                SIGMASTUDIOTYPE_8_24

#endif
