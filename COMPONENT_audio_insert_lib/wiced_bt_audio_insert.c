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

#include "wiced_bt_audio_insert.h"
#include "wiced_bt_trace.h"
#include "i2s_aud_inject.h"
#include "audio_insert_sco.h"
#include "audio_insert_audio.h"

typedef struct
{
    wiced_bool_t    initialized;
} wiced_bt_audio_insert_cb_t;

static wiced_bt_audio_insert_cb_t wiced_bt_audio_insert_cb = {0};

#ifdef CYW20721B2
static wiced_bt_audio_insert_callback_t *wiced_bt_audio_insert_app_cb = NULL;

/*
 * Callback function for I2S Audio Inject Module.
 */
static void wiced_bt_audio_insert_i2s_aud_inject_callback(i2s_aud_inject_event_t event,
        i2s_aud_inject_event_data_t *p_data)
{
    wiced_bt_audio_insert_event_t wiced_event;
    wiced_bt_audio_insert_event_data_t wiced_event_data;

    switch (event)
    {
    /* i2s aud inject request to fill hardware FIFO with audio data */
    case I2S_AUD_INJECT_EVT_FILL_FIFO:
        wiced_event = WICED_BT_AUDIO_INSERT_EVT_DATA_REQ;
        wiced_event_data.data_req.p_data_in     = p_data->a2dp_samples.p_source;
        wiced_event_data.data_req.p_data_out    = p_data->a2dp_samples.p_finalOutput;
        wiced_event_data.data_req.bufferSize    = p_data->a2dp_samples.bufferSize;
        break;

    /* i2s aud inject indicates the sample rate for the pending audio injection */
    case I2S_AUD_INJECT_EVT_AUDIO_INFO:
        wiced_event = WICED_BT_AUDIO_INSERT_EVT_AUDIO_INFO;
        wiced_event_data.audio_info.nb_samples  = p_data->a2dp_info.bufferSize;
        wiced_event_data.audio_info.sample_rate = p_data->a2dp_info.sampleRate;
        break;

    /* Lite Host indicates start SCO audio injection */
    case LITE_HOST_EVT_START_SCO_INJECT:
        wiced_event = WICED_BT_AUDIO_INSERT_EVT_SCO;
        wiced_event_data.sco_info.nb_samples    = p_data->sco_start.bufferSize;
        wiced_event_data.sco_info.sample_rate   = p_data->sco_start.sampleRate;
        break;
    default:
        return;
    }

    if (wiced_bt_audio_insert_app_cb)
    {
        (*wiced_bt_audio_insert_app_cb)(wiced_event, &wiced_event_data);
    }

    switch (event)
    {
    /* After the audio was inserted, assign p_finalOutput */
    case I2S_AUD_INJECT_EVT_FILL_FIFO:
        p_data->a2dp_samples.p_source = wiced_event_data.data_req.p_data_in;
        p_data->a2dp_samples.p_finalOutput = wiced_event_data.data_req.p_data_out;
        break;
    default:
        break;
    }
}
#endif

/*
 * wiced_bt_audio_insert_init
 */
#ifdef CYW20721B2
wiced_result_t wiced_bt_audio_insert_init(wiced_bt_audio_insert_callback_t *p_callback)
{
    wiced_result_t status;

    WICED_BT_TRACE("%s: p_callback:0x%x\n", __FUNCTION__, p_callback);

    /* Check parameter. */
    if (p_callback == NULL)
    {
        return WICED_BADARG;
    }

    /* Call the I2S Audio Insert Init function. */
    status = i2s_aud_inject_init(&wiced_bt_audio_insert_i2s_aud_inject_callback);
    if (status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("ERR: %s: i2s_aud_inject_init failed status:%d\n", __FUNCTION__, status);
        return status;
    }

    /* Save information used later.
     * wiced_bt_audio_insert_app_cb is a translation layer for B2 version.
     */
    wiced_bt_audio_insert_app_cb = p_callback;

    return status;
}
#else
wiced_result_t wiced_bt_audio_insert_init(wiced_bt_audio_insert_callback_t *p_callback)
{
    wiced_result_t status;

    WICED_BT_TRACE("%s: p_callback:0x%x\n", __FUNCTION__, p_callback);

    /* Call the I2S Audio Insert Init function.
     * Note that we can pass the application's callback function because the FW and Wiced
     * APIs use the exact definitions (definitions and structures)
     */
    status = i2s_aud_inject_init((i2s_aud_inject_callback_t *)p_callback);
    if (status != WICED_BT_SUCCESS)
        WICED_BT_TRACE("ERR: %s: i2s_aud_inject_init failed status:%d\n", __FUNCTION__, status);

    return status;
}
#endif

#if 1
/**
 * wiced_bt_audio_insert_init_new
 *
 * Initialize the WiCED BT Audio Insertion Module.
 */
void wiced_bt_audio_insert_init_new(void)
{
    /* Check state to prevent duplicated initialization. */
    if (wiced_bt_audio_insert_cb.initialized)
    {
        return;
    }

    /* Initialize the voice stream insertion module. */
    audio_insert_sco_init();

    /* Initialize the audio stream insertion module. */
    audio_insert_audio_init();

    wiced_bt_audio_insert_cb.initialized = WICED_TRUE;
}
#endif

/**
 * wiced_bt_audio_insert_stop
 *
 * Stop the audio insertion
 *
 * @param type - refer to wiced_bt_audio_insert_type_t
 */
void wiced_bt_audio_insert_stop(wiced_bt_audio_insert_type_t type)
{
    switch (type)
    {
    case WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC:
        audio_insert_sco_stop(WICED_TRUE, WICED_FALSE);
        break;

    case WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK:
        audio_insert_sco_stop(WICED_FALSE, WICED_TRUE);
        break;

    case WICED_BT_AUDIO_INSERT_TYPE_AUDIO:
        audio_insert_audio_stop();
        break;

    default:
        break;
    }
}

/**
 * wiced_bt_audio_insert_start
 *
 * Start the audio insertion
 *
 * @param p_config - refer to wiced_bt_audio_insert_config_t
 *
 */
void wiced_bt_audio_insert_start(wiced_bt_audio_insert_config_t *p_config)
{
    /* Check parameter. */
    if (p_config == NULL)
    {
        return;
    }

    switch (p_config->type)
    {
    /* Insertion data will be added in the Microphone data. */
    case WICED_BT_AUDIO_INSERT_TYPE_SCO_MIC:
        audio_insert_sco_start(WICED_TRUE,
                               WICED_FALSE,
                               &p_config->insert_data);
        break;

    /* Insertion data will be added in the Voice Call Speaker data. */
    case WICED_BT_AUDIO_INSERT_TYPE_SCO_SPK:
        audio_insert_sco_start(WICED_FALSE,
                               WICED_TRUE,
                               &p_config->insert_data);
        break;

    /* Insertion data will be added in the audio data (streaming or playback). */
    case WICED_BT_AUDIO_INSERT_TYPE_AUDIO:
        audio_insert_audio_start(p_config->multiple_device,
                                 p_config->p_sample_rate,
                                 &p_config->insert_data);
        break;

    default:
        break;
    }
}
