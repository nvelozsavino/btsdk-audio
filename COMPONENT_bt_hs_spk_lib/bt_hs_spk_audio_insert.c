/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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

/*
 * This file implement functions to Insert Audio (thanks to the Wiced Audio Insert library).
 * It allows the application to Play a (locally stored) sound.
 */
#include "bt_hs_spk_control.h"
#include "wiced.h"
#include "wiced_bt_audio_insert.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "bt_hs_spk_audio_insert.h"
#include "bt_hs_spk_handsfree.h"
#include "bt_hs_spk_audio.h"
#include "wiced_audio_manager.h"

/*
 * Definitions
 */
#ifndef APP_TRACE_DBG
#define APP_TRACE_DBG(format, ...)  WICED_BT_TRACE("%s " format, __FUNCTION__, ##__VA_ARGS__)
#endif
#ifndef APP_TRACE_ERR
#define APP_TRACE_ERR(format, ...)  WICED_BT_TRACE("Err: %s " format, __FUNCTION__, ##__VA_ARGS__)
#endif

typedef struct
{
    wiced_bool_t audio_insert_active;
    wiced_timer_t timer;
    wiced_bool_t sco_hook_registered;
} bt_hs_spk_audio_insert_cb_t;


/*
 * Global variables
 */
static bt_hs_spk_audio_insert_cb_t bt_hs_spk_audio_insert_cb;

static const int16_t sine_wave_mono[32] =
{
#if 1
    0x0000, 0x18f8, 0x30f8, 0x471c, 0x5a82, 0x6a6d, 0x7641, 0x7d89,
    0x7fff, 0x7d89, 0x7641, 0x6a6d, 0x5a82, 0x471c, 0x30f8, 0x18f8,
    0x0000, 0xe708, 0xcf05, 0xb8e4, 0xa57e, 0x9593, 0x898f, 0x8277,
    0x8001, 0x8277, 0x898f, 0x9593, 0xa57e, 0xb8e4, 0xcf05, 0xe708
#else
        0, 6392, 12539, 18204, 23170, 27245, 30273, 32137,
        32767, 32137, 30273, 27245, 23170, 18204, 12539, 6392,
        0, -6392, -12539, -18204, -23170, -27245, -30273, -32137,
        -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6392
#endif
};

static const int16_t sine_wave_stereo[64] =
{
    0x0000, 0x0000, 0x18f8, 0x18f8, 0x30f8, 0x30f8, 0x471c, 0x471c,
    0x5a82, 0x5a82, 0x6a6d, 0x6a6d, 0x7641, 0x7641, 0x7d89, 0x7d89,
    0x7fff, 0x7fff, 0x7d89, 0x7d89, 0x7641, 0x7641, 0x6a6d, 0x6a6d,
    0x5a82, 0x5a82, 0x471c, 0x471c, 0x30f8, 0x30f8, 0x18f8, 0x18f8,
    0x0000, 0x0000, 0xe708, 0xe708, 0xcf05, 0xcf05, 0xb8e4, 0xb8e4,
    0xa57e, 0xa57e, 0x9593, 0x9593, 0x898f, 0x898f, 0x8277, 0x8277,
    0x8001, 0x8001, 0x8277, 0x8277, 0x898f, 0x898f, 0x9593, 0x9593,
    0xa57e, 0xa57e, 0xb8e4, 0xb8e4, 0xcf05, 0xcf05, 0xe708, 0xe708
};

/*
 * Local functions
 */
static void bt_hs_spk_audio_insert_timer_callback(uint32_t param);
static void bt_hs_spk_audio_insert_sco_event_callback(void);
static void bt_hs_spk_audio_insert_a2dp_status_callback(void);

/*
 * bt_hs_spk_audio_insert_init
 */
wiced_result_t bt_hs_spk_audio_insert_init(void)
{
    wiced_result_t status;

    APP_TRACE_DBG("\n");

    memset(&bt_hs_spk_audio_insert_cb, 0, sizeof(bt_hs_spk_audio_insert_cb));

    status = wiced_init_timer(&bt_hs_spk_audio_insert_cb.timer,
            bt_hs_spk_audio_insert_timer_callback, 0, WICED_SECONDS_TIMER);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_init_timer failed status:%d\n", status);
        return status;
    }

    wiced_bt_audio_insert_init_new();

    /* Register a SCO Status callback (to know when SCO is active/inactive) */
    bt_audio_hfp_register_sco_event(bt_hs_spk_audio_insert_sco_event_callback);

    /* Register an A2DP Stream Status callback (to know when A2DP Stream is active/inactive) */
    bt_hs_spk_audio_a2dp_status_register(bt_hs_spk_audio_insert_a2dp_status_callback);

    return status;
}

static void bt_hs_spk_audio_insert_start_sco(void)
{
    wiced_bt_audio_insert_config_t audio_insert_config = {0};

    audio_insert_config.type                                                = WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK;
    audio_insert_config.multiple_device                                     = WICED_FALSE;
    audio_insert_config.p_sample_rate                                       = 0;
    audio_insert_config.insert_data.p_source                                = (int16_t *) sine_wave_mono;
    audio_insert_config.insert_data.len                                     = sizeof(sine_wave_mono);
    audio_insert_config.insert_data.overwrite                               = WICED_TRUE;
    audio_insert_config.insert_data.loop                                    = WICED_TRUE;
    audio_insert_config.insert_data.volume_reduce_factor_insert             = 10;
    audio_insert_config.insert_data.volume_reduce_factor_original           = 1;
    audio_insert_config.insert_data.p_source_data_exhausted_callback        = NULL;
    audio_insert_config.insert_data.stop_insertion_when_source_exhausted    = WICED_FALSE;

    wiced_bt_audio_insert_start(&audio_insert_config);

    bt_hs_spk_audio_insert_cb.sco_hook_registered = WICED_TRUE;
    bt_hs_spk_audio_insert_cb.audio_insert_active = WICED_TRUE;
}

static void bt_hs_spk_audio_insert_start_audio(uint32_t *p_sample_rate)
{
    wiced_bt_audio_insert_config_t audio_insert_config = {0};
    audio_config_t audio_config;

    audio_insert_config.type                                                = WICED_BT_AUDIO_INSERT_TYPE_AUDIO;
    audio_insert_config.multiple_device                                     = WICED_FALSE;
    audio_insert_config.p_sample_rate                                       = p_sample_rate;
    audio_insert_config.insert_data.p_source                                = (int16_t *) sine_wave_stereo;
    audio_insert_config.insert_data.len                                     = sizeof(sine_wave_stereo);
    audio_insert_config.insert_data.overwrite                               = WICED_FALSE;
    audio_insert_config.insert_data.loop                                    = WICED_TRUE;
    audio_insert_config.insert_data.volume_reduce_factor_insert             = 10;
    audio_insert_config.insert_data.volume_reduce_factor_original           = 4;
    audio_insert_config.insert_data.p_source_data_exhausted_callback        = NULL;
    audio_insert_config.insert_data.stop_insertion_when_source_exhausted    = WICED_FALSE;

    wiced_bt_audio_insert_start(&audio_insert_config);

    /* If A2DP is NOT active, we must configure and start a dedicated Audio Manager Stream */
    if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
    {
        audio_config.sr = *audio_insert_config.p_sample_rate;
        audio_config.channels = 1;
        audio_config.mic_gain = 0;
        audio_config.bits_per_sample = 16;
        audio_config.volume = 5;
        audio_config.sink = AM_HEADPHONES;

        bt_hs_spk_audio_audio_manager_stream_start(&audio_config);
    }

    bt_hs_spk_audio_insert_cb.audio_insert_active = WICED_TRUE;
}

/*
 * bt_hs_spk_audio_insert_start
 */
wiced_result_t bt_hs_spk_audio_insert_start(uint32_t *p_sample_rate, uint32_t duration_sec)
{
    wiced_result_t status;
    wiced_bt_audio_insert_config_t audio_insert_config = {0};

    /* Check if audio Insert is active */
    if (bt_hs_spk_audio_insert_cb.audio_insert_active)
    {
        APP_TRACE_ERR("Audio Insert already active\n");
        return WICED_BT_BADARG;
    }

    if (duration_sec == 0)
    {
        APP_TRACE_ERR("duration_sec is 0\n");
        return WICED_BT_BADARG;
    }

    if (p_sample_rate == NULL)
    {
        APP_TRACE_ERR("p_sample_rate cannot be NULL\n");
        return WICED_BT_BADARG;
    }

    status = wiced_start_timer(&bt_hs_spk_audio_insert_cb.timer, duration_sec);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_start_timer failed status:%d\n", status);
        return status;
    }

    /* If SCO is active, the Audio Insert must be done via the SCO Hook */
    if (bt_hs_spk_handsfree_sco_connection_check(NULL) == WICED_TRUE)
    {
        bt_hs_spk_audio_insert_start_sco();
    }
    else
    {
        bt_hs_spk_audio_insert_start_audio(p_sample_rate);
    }

    return WICED_BT_SUCCESS;
}

/*
 * bt_hs_spk_audio_insert_stop
 */
wiced_result_t bt_hs_spk_audio_insert_stop(void)
{
    wiced_result_t status;

    APP_TRACE_DBG("\n");

    if (bt_hs_spk_audio_insert_cb.audio_insert_active == WICED_FALSE)
    {
        APP_TRACE_ERR("Audio Insert not active\n");
        return WICED_BT_ERROR;
    }

    bt_hs_spk_audio_insert_cb.audio_insert_active = WICED_FALSE;

    status = wiced_stop_timer(&bt_hs_spk_audio_insert_cb.timer);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_stop_timer failed\n");
        return WICED_BT_ERROR;
    }

    if (bt_hs_spk_audio_insert_cb.sco_hook_registered)
    {
        APP_TRACE_DBG("Deregister SCO Hook\n");
        /* If the Audio Insertion was done via the SCO Hook, deregister it */
        wiced_bt_audio_insert_stop(WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK);

        bt_hs_spk_audio_insert_cb.sco_hook_registered = WICED_FALSE;
        status = WICED_BT_SUCCESS;
    }
    else
    {
        APP_TRACE_DBG("Stop Audio Insert\n");
        /* If the Audio Insertion was done via the Audio Insert library, stop it */
        wiced_bt_audio_insert_stop(WICED_BT_AUDIO_INSERT_TYPE_AUDIO);

        /* If a dedicated Audio Manager Stream (to control external codec) was used */
        if (bt_hs_spk_audio_streaming_check(NULL) != WICED_ALREADY_CONNECTED)
        {
            bt_hs_spk_audio_audio_manager_stream_stop();
        }
    }

    return status;
}

/*
 * bt_hs_spk_audio_insert_timer_callback
 */
static void bt_hs_spk_audio_insert_timer_callback(uint32_t param)
{
    bt_hs_spk_audio_insert_stop();
}

/*
 * bt_hs_spk_audio_insert_sco_event_callback
 */
static void bt_hs_spk_audio_insert_sco_event_callback(void)
{
    /* Check if audio Insert is active */
    if (bt_hs_spk_audio_insert_cb.audio_insert_active)
    {
        APP_TRACE_DBG("Audio Insert active. Stop it\n");
        bt_hs_spk_audio_insert_stop();
    }
}

/*
 * bt_hs_spk_audio_insert_a2dp_status_callback
 */
static void bt_hs_spk_audio_insert_a2dp_status_callback(void)
{
    /* Check if audio Insert is active */
    if (bt_hs_spk_audio_insert_cb.audio_insert_active)
    {
        APP_TRACE_DBG("Audio Insert active. Stop it\n");
        bt_hs_spk_audio_insert_stop();
    }
}
