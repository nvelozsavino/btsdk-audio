/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file implements the utility functions for handsfree
 */

#include <stdlib.h>
#include <string.h>
#include "wiced_audio_manager.h"
#include "bt_hs_spk_handsfree.h"


/**
 *
 * Volume levels passed from the application to Audio Manager should be in the range 0 to 10
 * calculating from 0 to 15 levels to 0 to 10 levels
 *
 * @param           int32_t  : vol from app.
 *
 * @return          volume in AM level
 */
int32_t bt_hs_spk_handsfree_utils_hfp_volume_to_am_volume(int32_t vol)
{
    uint32_t remainder;
    int32_t am_level;

    am_level    = (vol * AM_VOL_LEVEL_HIGH) / HFP_VOLUME_HIGH;
    remainder   = (vol * AM_VOL_LEVEL_HIGH) % HFP_VOLUME_HIGH;

    if (remainder >= AM_VOL_LEVEL_HIGH)
    {
        am_level++;
    }

    return am_level;
}
