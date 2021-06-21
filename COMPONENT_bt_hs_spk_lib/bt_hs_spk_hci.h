/*
 * bt_hs_spk_hci.h
 *
 *  Created on: Jun 18, 2021
 *      Author: otc_portatil
 */

#ifndef BT_HS_SPK_HCI_H
#define BT_HS_SPK_HCI_H
#include "wiced_bt_hfp_hf_int.h"
#include "hci_control_api.h"


/* data associated with HF_OPEN_EVT */
typedef struct
{
    BD_ADDR             bd_addr;
    uint8_t             status;
} hci_control_hfp_hf_open_t;

/* data associated with AT command response event */
typedef struct
{
    uint16_t            num;
    char                str[WICED_BT_HFP_HF_MAX_AT_CMD_LEN];
} hci_control_hfp_hf_value_t;

/* data associated with HF_CONNECTED_EVT */
typedef struct
{
    uint32_t           peer_features;
    uint8_t            profile_selected;
} hci_control_hfp_hf_connect_t;

/* union of data associated with HS callback */
typedef union
{
    hci_control_hfp_hf_open_t    open;
    hci_control_hfp_hf_connect_t conn;
    hci_control_hfp_hf_value_t   val;
} hci_control_hfp_hf_event_t;

typedef void (*hci_control_send_hf_event_t)(uint16_t evt, uint16_t handle, hci_control_hfp_hf_event_t *p_data);



#endif /* BT_HS_SPK_HCI_H */
