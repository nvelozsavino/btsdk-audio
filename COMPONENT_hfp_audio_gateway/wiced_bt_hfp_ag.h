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
 * This is the public interface file for the handsfree profile Audio Gateway(AG) subsystem of
 * HCI_CONTROL application.
 */
#ifndef WICED_BT_HFP_AG_H
#define WICED_BT_HFP_AG_H

#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_timer.h"

/******************************************************
 *                     Constants
 ******************************************************/
#define HFP_VERSION_1_5                 0x0105
#define HFP_VERSION_1_6                 0x0106
#define HFP_VERSION_1_7                 0x0107
#define HFP_AG_VERSION                  HFP_VERSION_1_6

/* HF feature masks */
#define HFP_HF_FEAT_ECNR                0x00000001   /* Echo cancellation and/or noise reduction */
#define HFP_HF_FEAT_3WAY                0x00000002   /* Call waiting and three-way calling */
#define HFP_HF_FEAT_CLIP                0x00000004   /* Caller ID presentation capability  */
#define HFP_HF_FEAT_VREC                0x00000008   /* Voice recoginition activation capability  */
#define HFP_HF_FEAT_RVOL                0x00000010   /* Remote volume control capability  */
#define HFP_HF_FEAT_ECS                 0x00000020   /* Enhanced Call Status  */
#define HFP_HF_FEAT_ECC                 0x00000040   /* Enhanced Call Control  */
#define HFP_HF_FEAT_CODEC               0x00000080   /* Codec negotiation */
#define HFP_HF_FEAT_HF_IND              0x00000100   /* HF Indicators */
#define HFP_HF_FEAT_ESCO                0x00000200   /* eSCO S4 ( and T2 ) setting supported */

/* AG feature masks */
#define HFP_AG_FEAT_3WAY                0x00000001   /* Three-way calling */
#define HFP_AG_FEAT_ECNR                0x00000002   /* Echo cancellation and/or noise reduction */
#define HFP_AG_FEAT_VREC                0x00000004   /* Voice recognition */
#define HFP_AG_FEAT_INBAND              0x00000008   /* In-band ring tone */
#define HFP_AG_FEAT_VTAG                0x00000010   /* Attach a phone number to a voice tag */
#define HFP_AG_FEAT_REJECT              0x00000020   /* Ability to reject incoming call */
#define HFP_AG_FEAT_ECS                 0x00000040   /* Enhanced Call Status */
#define HFP_AG_FEAT_ECC                 0x00000080   /* Enhanced Call Control */
#define HFP_AG_FEAT_EXTERR              0x00000100   /* Extended error codes */
#define HFP_AG_FEAT_CODEC               0x00000200   /* Codec Negotiation */
#define HFP_AG_FEAT_HF_IND              0x00000400   /* HF Indicators */
#define HFP_AG_FEAT_ESCO                0x00000800   /* eSCO S4 ( and T2 ) setting supported */

/* SCO Codec Types */
#define HFP_CODEC_CVSD                  0x0001
#define HFP_CODEC_MSBC                  0x0002

/* ASCII charcter string of arguments to the AT command or response */
#define HFP_AG_AT_MAX_LEN               200

/* HFP VGS and VGM Max/Min value */
#define HFP_VGM_VGS_MAX                 15
#define HFP_VGM_VGS_MIN                 0

#undef  BTM_WBS_INCLUDED
#define BTM_WBS_INCLUDED                TRUE

/* type for each service control block */
/* Handsfree device control block */
typedef struct
{
#define     HFP_AG_STATE_IDLE       0
#define     HFP_AG_STATE_OPENING    1
#define     HFP_AG_STATE_OPEN       2
#define     HFP_AG_STATE_CLOSING    3

    uint8_t             state;                  /* state machine state */
    uint16_t            app_handle;             /* Handle used to identify with the app */

    uint8_t             b_is_initiator;         /* initiator of the connection ( true ) or acceptor ( false ) */
    BOOLEAN             b_slc_is_up;            /* set to TRUE when service level connection up */
    BOOLEAN             b_call_is_up;           /* set to TRUE when phone call connection up for HSP */
    uint16_t            hf_profile_uuid;        /* Used to store HS/HF profile ID for Initiator Role */

    uint16_t            rfc_serv_handle;        /* RFCOMM server handle */
    uint16_t            rfc_conn_handle;        /* RFCOMM handle of connected service */
    uint8_t             hf_scn;                 /* HF's scn */
    BD_ADDR             hf_addr;                /* HF's bd address */

    char                res_buf[HFP_AG_AT_MAX_LEN + 1];     /* temp parsing buffer */
    int                 res_len;                /* length of data in temp buffer */

    wiced_bt_sdp_discovery_db_t *p_sdp_discovery_db;                /* pointer to discovery database */

    uint32_t            hf_features;            /* HF device features */
    uint16_t            hf_version;             /* HF device profile version */

#if (BTM_WBS_INCLUDED == TRUE )
    wiced_timer_t       cn_timer;               /* codec negotiation timer */

    BOOLEAN             peer_supports_msbc;     /* TRUE if peer supports mSBC */
    BOOLEAN             msbc_selected;          /* TRUE if we have selected mSBC */
#endif

    uint16_t            sco_idx;                /* SCO handle */
    BOOLEAN             b_sco_opened;           /* set to TRUE when SCO connection is open */

    BOOLEAN             retry_with_sco_only;    /* ind to try with SCO only if eSCO fails */

    BOOLEAN             clip_enabled;           /* set to TRUE if HF enables CLIP reporting */
    BOOLEAN             cmer_enabled;           /* set to TRUE if HF enables CMER reporting */
    BOOLEAN             cmee_enabled;           /* set to TRUE if HF enables CME ERROR reporting */

} hfp_ag_session_cb_t;

/* HS settings */
typedef struct
{
    uint8_t        spk_vol;
    uint8_t        mic_vol;
    BOOLEAN        ecnr_enabled;
} hci_control_ag_settings_t;

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
/*
 * Start the handsfree service. This function must be called once to initialize
 * the handsfree device.  Internally function starts SDP and RFCOMM processing.
 * This function must be called before other function in the HF API called.
 */
extern void hfp_ag_startup( hfp_ag_session_cb_t *p_scb, uint8_t num_scb, uint32_t features );

/*
 * Opens a connection to a audio gateway.  When connection is open callback
 * function is called with a HCI_CONTROL_HF_EVENT_OPEN event indicating
 * success or failure of the operation. Only the service level data connection
 * is opened. The audio connection is not opened.
 */
extern void hfp_ag_connect ( BD_ADDR bd_addr );

/*
 * Close the current connection to a audio gateway.  Any current audio
 * connection will also be closed.
 */
extern void hfp_ag_disconnect( uint16_t handle );

/*
 * Opens an audio connection to the currently connected audio gateway specified
 * by the handle.
 */
extern  void hfp_ag_audio_open( uint16_t handle );

/*
 * Close the currently active audio connection to a audio gateway specified by
 * the handle. The data connection remains opened.
 */
extern void hfp_ag_audio_close( uint16_t handle );

extern void hfp_ag_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/* AT functions */
extern void hfp_ag_send_BCS_to_hf (hfp_ag_session_cb_t *p_scb);
extern void hfp_ag_send_BVRA_to_hf (hfp_ag_session_cb_t *p_scb, BOOLEAN b_is_active);
extern void hfp_ag_send_RING_to_hf (hfp_ag_session_cb_t *p_scb);
extern uint8_t hfp_ag_send_VGM_to_hf (hfp_ag_session_cb_t *p_scb, INT16 gain);
extern uint8_t hfp_ag_send_VGS_to_hf (hfp_ag_session_cb_t *p_scb, INT16 gain);

#ifdef __cplusplus
}
#endif

#endif /* WICED_BT_HFP_AG_H */
