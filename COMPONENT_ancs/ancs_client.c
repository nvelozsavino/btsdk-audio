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
* BLE Client for Apple Notification Center Service (ANCS)
*
* During initialization the app registers with LE stack to receive various
* notifications including bonding complete, connection status change and
* peer notification.  When device is successfully bonded, application saves
* peer's Bluetooth Device address to the NVRAM and starts GATT service
* discovery.  The ANCS UUIDs are published at
* https://developer.apple.com/library/IOS/documentation/CoreBluetooth/Reference/AppleNotificationCenterServiceSpecification/Specification/Specification.html
* If service discovery is successful application writes into
* appropriate Characteristic Client Configuration descriptor registering
* for notifications from the iOS device.  Received messages are printed
* out to the device output.
*
* Features demonstrated
*  - Registration with LE stack for various events
*  - performing GATT service discovery
*  - working with ANCS service on iOS device
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Start tracing to monitor the activity (see Kit Guide for details)
* 4. Pair with an iOS device (please note that iOS does not like to pair with a device, use some app instead)
* 5. Send an SMS or generate incoming call to you iOS device
*
*/
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ancs.h"
#include "wiced_memory.h"
#include "wiced_timer.h"

#include "string.h"

#define ANCS_CLIENT_DEBUG_ENABLE        1
#define ANCS_ADDITIONAL_TRACE           0

/******************************************************
 *                      Constants
 ******************************************************/

// ANCS get notification attribute retry timeour
#define ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT    1   // second

/// max notifications to queue
#ifndef ANCS_MAX_QUEUED_NOTIFICATIONS
#define ANCS_MAX_QUEUED_NOTIFICATIONS                   20
#endif // ANCS_MAX_QUEUED_NOTIFICATIONS

// 7905F431-B5CE-4E99-A40F-4B1E122D00D0
const char ANCS_SERVICE[]             = {0xD0, 0x00, 0x2D, 0x12, 0x1E, 0x4B, 0x0F, 0xA4, 0x99, 0x4E, 0xCE, 0xB5, 0x31, 0xF4, 0x05, 0x79};

// Notification Source: UUID 9FBF120D-6301-42D9-8C58-25E699A21DBD (notifiable)
const char ANCS_NOTIFICATION_SOURCE[] = {0xBD, 0x1D, 0xA2, 0x99, 0xE6, 0x25, 0x58, 0x8C, 0xD9, 0x42, 0x01, 0x63, 0x0D, 0x12, 0xBF, 0x9F};

// Control Point: UUID 69D1D8F3-45E1-49A8-9821-9BBDFDAAD9D9 (writeable with response)
const char ANCS_CONTROL_POINT[]       = {0xD9, 0xD9, 0xAA, 0xFD, 0xBD, 0x9B, 0x21, 0x98, 0xA8, 0x49, 0xE1, 0x45, 0xF3, 0xD8, 0xD1, 0x69};

// Data Source: UUID 22EAC6E9-24D6-4BB5-BE44-B36ACE7C7BFB (notifiable)
const char ANCS_DATA_SOURCE[]         = {0xFB, 0x7B, 0x7C, 0xCE, 0x6A, 0xB3, 0x44, 0xBE, 0xB5, 0x4B, 0xD6, 0x24, 0xE9, 0xC6, 0xEA, 0x22};

// following is the list of notification attributes that we are going
// to request.  Compile out attribute of no interest
uint8_t  ancs_client_notification_attribute[] =
{
//    ANCS_NOTIFICATION_ATTR_ID_APP_ID,
    ANCS_NOTIFICATION_ATTR_ID_TITLE,
//    ANCS_NOTIFICATION_ATTR_ID_SUBTITLE,
    ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE,
    ANCS_NOTIFICATION_ATTR_ID_MESSAGE,
//    ANCS_NOTIFICATION_ATTR_ID_DATE,
    ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL,
    ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL,
    0
};
// Maximum length we are going to request.  The values are valid for
// title subtitle and message.  The number of elements should match number
// of elements in the ancs_client_notification_attribute above
uint8_t  ancs_client_notification_attribute_length[] =
{
//    0,
    20,
//    20,
    0,
    255,
//    0,
    0,
    0,
    0
};

#ifdef WICED_BT_TRACE_ENABLE
#ifdef ANCS_ADDITIONAL_TRACE
static char *EventId[] =
{
    "Added",
    "Modified",
    "Removed",
    "Unknown"
};

#define ANCS_CATEGORY_ID_MAX    12
static char *CategoryId[] =
{
    "Other",
    "IncomingCall",
    "MissedCall",
    "Voicemail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "HealthAndFitness",
    "BusinessAndFinance",
    "Location",
    "Entertainment",
    "Unknown"
};

static char *NotificationAttributeID[] =
{
    "AppIdentifier",
    "Title",
    "Subtitle",
    "Message",
    "MessageSize",
    "Date",
    "PositiveActLabel",
    "NegativeActLabel",
    "Unknown"
};
#endif
#endif

#define ANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES     0
#define ANCS_COMMAND_ID_GET_APP_ATTRIBUTES              1
#define ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION     2

// service discovery states
enum
{
    ANCS_CLIENT_STATE_IDLE                                           = 0x00,
    ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD              = 0x01,
    ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD                      = 0x02,
    ANCS_CLIENT_STATE_WRITE_DATA_SOURCE_CCCD                         = 0x03,
    ANCS_CLIENT_STATE_WRITE_NOTIFICATION_SOURCE_CCCD                 = 0x04,
};

/******************************************************
 *                     Structures
 ******************************************************/

// ANCS event as the library passes to the application
typedef struct
{
    void *p_next;   // pointer to the next event when in the queue
    struct
    {
        wiced_bt_ancs_client_notification_data_basic_t  basic;
        wiced_bt_ancs_client_notification_data_info_t   info;
    } data;
} ancs_client_event_t;

// ANCS queued event description. Notification is queued while
// we are busy retrieving data from the current notification.
typedef struct
{
    void *p_next;
    struct
    {
        wiced_bt_ancs_client_notification_data_basic_t basic;
    } data;
} ancs_client_queued_event_t;

typedef struct t_ANCS_CLIENT
{
    wiced_bool_t init;

    uint8_t   state;
    uint8_t   notification_attribute_inx;
    uint16_t  conn_id;
    uint16_t  ancs_e_handle;
    uint16_t  notification_source_char_hdl;
    uint16_t  notification_source_val_hdl;
    uint16_t  notification_source_cccd_hdl;
    uint16_t  control_point_char_hdl;
    uint16_t  control_point_val_hdl;
    uint16_t  data_source_char_hdl;
    uint16_t  data_source_val_hdl;
    uint16_t  data_source_cccd_hdl;

    uint8_t *p_event_pool;
    ancs_client_event_t *p_first_event;

    uint16_t  data_left_to_read;
    uint16_t  data_source_buffer_offset;
    uint8_t   data_source_buffer[256];

    wiced_timer_t ancs_retry_timer;
    wiced_bt_ancs_client_event_handler_t *p_app_cb;
} ANCS_CLIENT;

/******************************************************
 *               Variables Definitions
 ******************************************************/
ANCS_CLIENT     ancs_client = {0};

/******************************************************
 *               Function Prototypes
 ******************************************************/
static void                     ancs_client_retry_timeout(uint32_t count);
static void                     ancs_client_send_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid, uint16_t s_handle, uint16_t e_handle);
static wiced_bt_gatt_status_t   ancs_client_send_next_get_notification_attributes_command(uint32_t uid);
static void                     ancs_client_set_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value);
static void                     ancs_client_stop(void);

#if (ANCS_CLIENT_DEBUG_ENABLE != 0)
#define ANCS_CLIENT_TRACE(format, ...) \
        WICED_BT_TRACE(format, ##__VA_ARGS__)
#else
#define ANCS_CLIENT_TRACE(...)
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * The application should call this function when BLE connection with a peer
 * device has been established.
 *
 * @param           p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t which includes the address and connection ID.
 * @return          none
 */
void wiced_bt_ancs_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ANCS_CLIENT_TRACE("%s %B%u\n", __func__, p_conn_status->bd_addr, p_conn_status->conn_id);
}

/*
 * Connection down event from the main application
 */
void wiced_bt_ancs_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ANCS_CLIENT_TRACE("%s %B%u\n", __func__, p_conn_status->bd_addr, p_conn_status->conn_id);

    if (ancs_client.conn_id == p_conn_status->conn_id)
    {
        ancs_client_stop();
    }
}

/**
 * wiced_bt_ancs_client_initialize
 *
 * Initialize the ANCS Client module.
 *
 * @param p_config  - Configuration
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ancs_client_initialize(wiced_bt_ancs_client_config_t *p_config)
{
    if (ancs_client.init)
    {
        return WICED_TRUE;
    }

    /* Creating a buffer pool for holding the peer devices's key info */
    ancs_client.p_event_pool = (uint8_t *) wiced_bt_create_pool(sizeof(ancs_client_queued_event_t),
                                                                ANCS_MAX_QUEUED_NOTIFICATIONS);

    if (ancs_client.p_event_pool == NULL)
    {
        return WICED_FALSE;
    }

    /* Initialize connection timer */
    if (wiced_init_timer(&ancs_client.ancs_retry_timer,
                         &ancs_client_retry_timeout,
                         0,
                         WICED_SECONDS_TIMER) != WICED_BT_SUCCESS)
    {
        return WICED_FALSE;
    }

    ancs_client.init = WICED_TRUE;
    ancs_client.p_app_cb = p_config->p_event_handler;

    return WICED_TRUE;
}

/**
 * wiced_bt_ancs_client_start
 *
 * Start search for ANCS characteristics.
 *
 * @param conn_id   : GATT Connection ID
 * @param s_handle  : Start handle value for GATT attribute operation
 * @param e_handle  : End handle value for GATT attribute operation
 *
 * @return  WICED_TRUE  : Success
 *          WICED_FALSE : Fail
 */
wiced_bool_t wiced_bt_ancs_client_start(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    ANCS_CLIENT_TRACE("[%s] (0x%04X, 0x%04X, 0x%04X)\n", __FUNCTION__, conn_id, s_handle, e_handle);

    if ((s_handle == 0) || (e_handle == 0))
        return WICED_FALSE;

    ancs_client.conn_id       = conn_id;
    ancs_client.ancs_e_handle = e_handle;
    ancs_client.state         = ANCS_CLIENT_STATE_IDLE;

    ancs_client_send_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, s_handle, e_handle);

    return WICED_TRUE;
}

static void ancs_client_stop(void)
{
    ancs_client_event_t *p_ancs_event = NULL;

    /* Stop timer. */
    if (wiced_is_timer_in_use(&ancs_client.ancs_retry_timer))
    {
        wiced_stop_timer(&ancs_client.ancs_retry_timer);
    }

    /* Reset information. - todo*/
    ancs_client.state                           = ANCS_CLIENT_STATE_IDLE;
    ancs_client.notification_attribute_inx      = 0;
    ancs_client.conn_id                         = 0;
    ancs_client.ancs_e_handle                   = 0;
    ancs_client.notification_source_char_hdl    = 0;
    ancs_client.notification_source_val_hdl     = 0;
    ancs_client.notification_source_cccd_hdl    = 0;
    ancs_client.control_point_char_hdl          = 0;
    ancs_client.control_point_val_hdl           = 0;
    ancs_client.data_source_char_hdl            = 0;
    ancs_client.data_source_val_hdl             = 0;
    ancs_client.data_source_cccd_hdl            = 0;
    ancs_client.data_left_to_read               = 0;
    ancs_client.data_source_buffer_offset       = 0;
    memset((void *) ancs_client.data_source_buffer, 0, sizeof(ancs_client.data_source_buffer));

    /* Clear queued events. */
    while (ancs_client.p_first_event)
    {
        p_ancs_event = ancs_client.p_first_event;
        ancs_client.p_first_event = ancs_client.p_first_event->p_next;

        wiced_bt_free_buffer((void *) p_ancs_event);
    }
}

/*
 * The function invoked on retry timeout retry sending attribute request
 */
static void ancs_client_retry_timeout(uint32_t count)
{
    wiced_bt_gatt_status_t status;

    ANCS_CLIENT_TRACE("%s\n", __FUNCTION__);

    /* Stop retry timer */
    wiced_stop_timer (&ancs_client.ancs_retry_timer);

    if (ancs_client.p_first_event != 0)
    {
        status = ancs_client_send_next_get_notification_attributes_command(ancs_client.p_first_event->data.basic.notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_CLIENT_TRACE("busy retrieve:%d\n", ancs_client.p_first_event->data.basic.notification_uid);
            wiced_start_timer(&ancs_client.ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
        }
    }
}

/*
 * Process discovery results from the stack.  We are looking for 3 characteristics
 * notification source, data source, and control point.  First 2 have client
 * configuration descriptors (CCCD).
 */
void wiced_bt_ancs_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    ANCS_CLIENT_TRACE("[%s]\n", __FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 16)
        {
            if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_NOTIFICATION_SOURCE, 16) == 0)
            {
                ancs_client.notification_source_char_hdl = p_char->handle;
                ancs_client.notification_source_val_hdl  = p_char->val_handle;
                ANCS_CLIENT_TRACE("notification source hdl:%04x-%04x", ancs_client.notification_source_char_hdl, ancs_client.notification_source_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_CONTROL_POINT, 16) == 0)
            {
                ancs_client.control_point_char_hdl = p_char->handle;
                ancs_client.control_point_val_hdl  = p_char->val_handle;
                ANCS_CLIENT_TRACE("control hdl:%04x-%04x", ancs_client.control_point_char_hdl, ancs_client.control_point_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_DATA_SOURCE, 16) == 0)
            {
                ancs_client.data_source_char_hdl = p_char->handle;
                ancs_client.data_source_val_hdl  = p_char->val_handle;
                ANCS_CLIENT_TRACE("data source hdl:%04x-%04x", ancs_client.data_source_char_hdl, ancs_client.data_source_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        // result for descriptor discovery, save appropriate handle based on the state
        if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD)
        {
            ancs_client.notification_source_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            ANCS_CLIENT_TRACE("notification_source_cccd_hdl hdl:%04x", ancs_client.notification_source_cccd_hdl);
        }
        else if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD)
        {
            ancs_client.data_source_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            ANCS_CLIENT_TRACE("data_source_cccd_hdl hdl:%04x", ancs_client.data_source_cccd_hdl);
        }
    }
}

/*
 * Process discovery complete event from the stack
 */
void wiced_bt_ancs_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;
    wiced_bt_ancs_client_event_data_t event_data;

    ANCS_CLIENT_TRACE("[%s] state:%d\n", __FUNCTION__, ancs_client.state);

    if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with ANCS characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((ancs_client.notification_source_char_hdl == 0) ||
            (ancs_client.notification_source_val_hdl == 0 ) ||
            (ancs_client.control_point_char_hdl == 0) ||
            (ancs_client.control_point_val_hdl == 0 ) ||
            (ancs_client.data_source_char_hdl == 0) ||
            (ancs_client.data_source_val_hdl == 0))
        {
            // something is very wrong
            ANCS_CLIENT_TRACE("[%s] failed\n", __FUNCTION__);
            ancs_client_stop();

            if (ancs_client.p_app_cb)
            {
                event_data.initialized.result = WICED_FALSE;

                (*ancs_client.p_app_cb)(WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED, &event_data);
            }

            return;
        }

        // search for descriptor from the characteristic characteristic until the end of the
        // service or until the start of the next characteristic
        end_handle = ancs_client.ancs_e_handle;
        if (ancs_client.control_point_char_hdl > ancs_client.notification_source_char_hdl)
            end_handle = ancs_client.control_point_char_hdl - 1;
        if ((ancs_client.data_source_char_hdl > ancs_client.notification_source_char_hdl) && (ancs_client.data_source_char_hdl < end_handle))
            end_handle = ancs_client.data_source_char_hdl - 1;

        ancs_client.state = ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD;
        ancs_client_send_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                 ancs_client.notification_source_val_hdl + 1, end_handle);
    }
    else if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD)
        {
            // search for descriptor from the characteristic characteristic until the end of the
            // service or until the handle of the next characteristic
            end_handle = ancs_client.ancs_e_handle;
            if (ancs_client.control_point_char_hdl > ancs_client.data_source_char_hdl)
                end_handle = ancs_client.control_point_char_hdl - 1;
            if ((ancs_client.notification_source_char_hdl > ancs_client.data_source_char_hdl) && (ancs_client.notification_source_char_hdl < end_handle))
                end_handle = ancs_client.notification_source_char_hdl - 1;

            ancs_client.state = ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD;
            ANCS_CLIENT_TRACE("send discover ancs_client_state:%02x %04x %04x\n", ancs_client.state, ancs_client.data_source_val_hdl + 1, end_handle - 1);
            ancs_client_send_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     ancs_client.data_source_val_hdl + 1, end_handle);
        }
        else if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD)
        {
            // done with descriptor discovery, register for notifications for data source by writing 1 into CCCD.
            ancs_client.state = ANCS_CLIENT_STATE_WRITE_DATA_SOURCE_CCCD;
            ancs_client_set_client_config_descriptor(p_data->conn_id, ancs_client.data_source_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
        }
    }
}

/**
 * wiced_bt_ancs_client_read_rsp
 *
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
}

/**
 * The application should call this function when it receives GATT Write Response
 * for the attribute handle which belongs to the ANCS service.
 *
 * @param           p_data  : pointer to a GATT operation complete data structure.
 * @return          none
 */
void wiced_bt_ancs_client_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_bt_ancs_client_event_data_t event_data;

    ANCS_CLIENT_TRACE("[%s] state:%02x\n", __FUNCTION__, ancs_client.state);

    // if we were writing 1 to notification source, still need to write 1 to data source
    if (ancs_client.state == ANCS_CLIENT_STATE_WRITE_DATA_SOURCE_CCCD)
    {
        ancs_client.state = ANCS_CLIENT_STATE_WRITE_NOTIFICATION_SOURCE_CCCD;
        ancs_client_set_client_config_descriptor(p_data->conn_id, ancs_client.notification_source_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
    }
    // if we were writing 1 to data source, done with initialization
    else if (ancs_client.state == ANCS_CLIENT_STATE_WRITE_NOTIFICATION_SOURCE_CCCD)
    {
        ancs_client.state = ANCS_CLIENT_STATE_IDLE;

        if (ancs_client.p_app_cb)
        {
            event_data.initialized.result = WICED_TRUE;

            (*ancs_client.p_app_cb)(WICED_BT_ANCS_CLIENT_EVENT_INITIALIZED, &event_data);
        }
    }
}

/*
 * Send command to the phone to get notification attributes.
 */
static wiced_bt_gatt_status_t ancs_client_send_next_get_notification_attributes_command(uint32_t uid)
{
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t  *p_write = (wiced_bt_gatt_value_t *)buf;
    uint8_t                *p_command = p_write->value;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ancs_client.control_point_val_hdl;
    p_write->offset   = 0;
    p_write->auth_req = GATT_AUTH_REQ_NONE;

    *p_command++ = ANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES;
    *p_command++ = uid & 0xff;
    *p_command++ = (uid >> 8) & 0xff;
    *p_command++ = (uid >> 16) & 0xff;
    *p_command++ = (uid >> 24) & 0xff;

    *p_command++ = ancs_client_notification_attribute[ancs_client.notification_attribute_inx];
    if (ancs_client_notification_attribute_length[ancs_client.notification_attribute_inx] != 0)
    {
        *p_command++ = ancs_client_notification_attribute_length[ancs_client.notification_attribute_inx];
        *p_command++ = 0;
    }
    p_write->len      = (uint8_t)(p_command - p_write->value);
    status = wiced_bt_gatt_send_write ( ancs_client.conn_id, GATT_WRITE, p_write );

    ANCS_CLIENT_TRACE("%s status:%d", __FUNCTION__, status);
    return status;
}

/**
 * The application calls this function to send the command to the phone to perform specified action.
 * The action command (for example answer the call, or clear notification, is sent as a response to
 * a notification. The UID of the notification is passed back in this function along with the action ID.
 *
 * @param           conn_id : Connection ID.
 * @param           uid : UID as received in the notification.
 * @param           action_id : Positive or Netgative action ID for the notification specified by UID.
 * @return          WICED_TRUE  : Success
 *                  WICED_FALSE : Fail
 */
wiced_bool_t wiced_ancs_client_send_remote_command(uint32_t uid, uint32_t action_id)
{
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t  *p_write = (wiced_bt_gatt_value_t *) buf;
    uint8_t                *p_command = p_write->value;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    ANCS_CLIENT_TRACE("%s uid:%d action:%d\n", __FUNCTION__, uid, action_id);

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ancs_client.control_point_val_hdl;
    p_write->offset   = 0;
    p_write->auth_req = GATT_AUTH_REQ_NONE;

    *p_command++ = ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION;
    *p_command++ = uid & 0xff;
    *p_command++ = (uid >> 8) & 0xff;
    *p_command++ = (uid >> 16) & 0xff;
    *p_command++ = (uid >> 24) & 0xff;

    *p_command++ = action_id;

    p_write->len      = (uint8_t)(p_command - p_write->value);
    status = wiced_bt_gatt_send_write(ancs_client.conn_id, GATT_WRITE, p_write);

    ANCS_CLIENT_TRACE("%s status:%d", __FUNCTION__, status);

    if (status == WICED_BT_GATT_SUCCESS)
        return WICED_TRUE;
    else
        return WICED_FALSE;
}

/*
 * Process Notification Source messages from the phone.
 * If it is a new or modified notification start reading attributes.
 */
static void ancs_client_process_notification_source(uint8_t *data, int len)
{
    wiced_bt_gatt_status_t status;
    ancs_client_event_t    *p_ancs_event;
    ancs_client_event_t    *p_prev = NULL;
    wiced_bt_ancs_client_event_data_t event_data;

    // Skip all pre-existing events
    if (data[1] & ANCS_EVENT_FLAG_PREEXISTING)
    {
        ANCS_CLIENT_TRACE("skipped preexisting event UID:%d\n", data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24));
        return;
    }

    switch (data[0])
    {
    case ANCS_EVENT_ID_NOTIFICATION_ADDED:
    case ANCS_EVENT_ID_NOTIFICATION_MODIFIED:
    case ANCS_EVENT_ID_NOTIFICATION_REMOVED:
        break;

    default:
        ANCS_CLIENT_TRACE("unknown command:%d\n", data[0]);
        return;
    }

    // if it is first notification, get the buffer to fill all information
    // if we are just queuing the notification, allocate small buffer from the pool
    if (ancs_client.p_first_event == NULL)
    {
        if ((p_ancs_event = (ancs_client_event_t *) wiced_bt_get_buffer(sizeof(ancs_client_event_t))) == NULL)
        {
            ANCS_CLIENT_TRACE("Failed to get buf\n");
            return;
        }
        memset (p_ancs_event, 0, sizeof(ancs_client_event_t));
    }
    else
    {
        if ((p_ancs_event = (ancs_client_event_t *) wiced_bt_get_buffer_from_pool((wiced_bt_buffer_pool_t *) ancs_client.p_event_pool)) == NULL)
        {
            ANCS_CLIENT_TRACE("Failed to get pool buf pool\n");
            return;
        }
        ANCS_CLIENT_TRACE("buf from pool:%d\n", p_ancs_event);
        memset (p_ancs_event, 0, sizeof(ancs_client_queued_event_t));
    }
    p_ancs_event->data.basic.notification_uid = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);

#ifdef ANCS_ADDITIONAL_TRACE
    ANCS_CLIENT_TRACE ("ANCS Notification EventID:%s EventFlags:%04x CategoryID:%s CategoryCount:%d UID:%04x",
            (uint32_t)(data[0] < ANCS_EVENT_ID_MAX) ? EventId[data[0]] : EventId[ANCS_EVENT_ID_MAX],
            data[1],
            (uint32_t)(data[2] < ANCS_CATEGORY_ID_MAX) ? CategoryId[data[2]] : CategoryId[ANCS_CATEGORY_ID_MAX],
            data[3],
            p_ancs_event->data.basic.notification_uid);
    if (len > 8)
        wiced_trace_array(&data[8], len - 8);
#endif


    ANCS_CLIENT_TRACE("notification type:%d, uuid:%d\n", data[0], p_ancs_event->data.basic.notification_uid);

    p_ancs_event->data.basic.command    = data[0];
    p_ancs_event->data.basic.flags      = data[1];
    p_ancs_event->data.basic.category   = data[2];

    // if we do not need to get details, and if there is nothing in the queue, can ship it out now
    if ((p_ancs_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_REMOVED) && (ancs_client.p_first_event == NULL))
    {
        if (ancs_client.p_app_cb)
        {
            event_data.notification.p_data = (wiced_bt_ancs_client_notification_data_t *) &p_ancs_event->data;
            (*ancs_client.p_app_cb)(WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION, &event_data);
        }

        wiced_bt_free_buffer(p_ancs_event);
        return;
    }
    // enqueue new event at the end of the queue
    if (ancs_client.p_first_event == NULL)
        ancs_client.p_first_event = p_ancs_event;
    else
    {
        for (p_prev = ancs_client.p_first_event; p_prev->p_next != NULL; p_prev = p_prev->p_next)
            ;
        p_prev->p_next = p_ancs_event;
    }

    if ((p_ancs_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_ADDED) || (p_ancs_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_MODIFIED))
    {
        // if we could not send previous request, need to wait for timer to expire.
        if (wiced_is_timer_in_use(&ancs_client.ancs_retry_timer))
        {
            return;
        }
        // if we are currently in process of dealing with another event just return
        if (ancs_client.p_first_event == p_ancs_event)
        {
            status = ancs_client_send_next_get_notification_attributes_command(p_ancs_event->data.basic.notification_uid);
            if (status == WICED_BT_GATT_BUSY)
            {
                // another GATT procedure is currently active, retry in a second
                ANCS_CLIENT_TRACE("busy retrieve:%d\n", p_ancs_event->data.basic.notification_uid);
                wiced_start_timer(&ancs_client.ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
            }
            else if (status != WICED_BT_GATT_SUCCESS)
            {
                ANCS_CLIENT_TRACE("ancs gatt failed:%02x uid:%d\n", status, p_ancs_event->data.basic.notification_uid);
                wiced_bt_free_buffer(p_ancs_event);
                if (ancs_client.p_first_event == p_ancs_event)
                {
                    ancs_client.p_first_event = NULL;
                }
                else
                {
                    p_prev->p_next = p_ancs_event;
                }
            }
        }
        else
        {
            ANCS_CLIENT_TRACE("will retrieve details later\n");
        }
    }
}

/*
 * Process additional message attributes.  The header file defines which attributes
 * we are asking for.
 */
static void ancs_client_process_event_attribute(uint8_t  *data, int len)
{
    uint8_t                 type = data[0];
    uint16_t                length = data[1] + (data[2] << 8);
    uint8_t *               p_event_data = &data[3];
    ancs_client_event_t     *p_event = ancs_client.p_first_event;
    ancs_client_queued_event_t *p_queued_event;
    wiced_bt_gatt_status_t  status;
    wiced_bt_ancs_client_event_data_t event_data;

    ANCS_CLIENT_TRACE("%s (0x%08X)\n", __FUNCTION__, ancs_client.p_first_event);

    ancs_client.data_left_to_read         = 0;
    ancs_client.data_source_buffer_offset = 0;

    switch(type)
    {
#ifdef ANCS_NOTIFICATION_ATTR_ID_APP_ID
    case ANCS_NOTIFICATION_ATTR_ID_APP_ID:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_TITLE
    case ANCS_NOTIFICATION_ATTR_ID_TITLE:
        memcpy(p_event->data.info.title, p_event_data, (length < sizeof(p_event->data.info.title) ? length : sizeof(p_event->data.info.title)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_SUBTITLE
    case ANCS_NOTIFICATION_ATTR_ID_SUBTITLE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE
    case ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_MESSAGE
    case ANCS_NOTIFICATION_ATTR_ID_MESSAGE:
        memcpy(p_event->data.info.message, p_event_data, (length < sizeof(p_event->data.info.message) ? length : sizeof(p_event->data.info.message)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_DATE
    case ANCS_NOTIFICATION_ATTR_ID_DATE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL
    case ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL:
        memcpy(p_event->data.info.positive_action_label, p_event_data, (length < sizeof(p_event->data.info.positive_action_label) ? length : sizeof(p_event->data.info.positive_action_label)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL
    case ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL:
        memcpy(p_event->data.info.negative_action_label, p_event_data, (length < sizeof(p_event->data.info.negative_action_label) ? length : sizeof(p_event->data.info.negative_action_label)));
        break;
#endif
    }

    // if we are not done with attributes, request the next one
    if (ancs_client_notification_attribute[++ancs_client.notification_attribute_inx] != 0)
    {
        status = ancs_client_send_next_get_notification_attributes_command(p_event->data.basic.notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_CLIENT_TRACE("busy retrieve:%d\n", p_event->data.basic.notification_uid);
            wiced_start_timer(&ancs_client.ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
        }
    }
    else
    {
        // Done with attributes for current event
        ancs_client.notification_attribute_inx = 0;

        p_event = ancs_client.p_first_event;

        if ((p_queued_event = ancs_client.p_first_event->p_next) != NULL)
        {
            if ((ancs_client.p_first_event = (ancs_client_event_t *) wiced_bt_get_buffer(sizeof(ancs_client_event_t))) == NULL)
            {
                ANCS_CLIENT_TRACE("Failed to get buf to copy\n");
                return;
            }
            memset (ancs_client.p_first_event, 0, sizeof(ancs_client_event_t));

            ancs_client.p_first_event->p_next = p_queued_event->p_next;

            memcpy((void *) &ancs_client.p_first_event->data.basic,
                   (void *) &p_queued_event->data.basic,
                   sizeof(wiced_bt_ancs_client_notification_data_basic_t));

            wiced_bt_free_buffer(p_queued_event);
        }
        else
        {
            ancs_client.p_first_event = NULL;
        }
        // ship current event to the application
        if (ancs_client.p_app_cb)
        {
            event_data.notification.p_data = (wiced_bt_ancs_client_notification_data_t *) &p_event->data;
            (*ancs_client.p_app_cb)(WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION, &event_data);
        }

        wiced_bt_free_buffer(p_event);

        // if next event in the queue is "Removed" ship it out right away
        while (ancs_client.p_first_event != NULL)
        {
            if (ancs_client.p_first_event->data.basic.command == ANCS_EVENT_ID_NOTIFICATION_REMOVED)
            {
                p_event = ancs_client.p_first_event;
                ancs_client.p_first_event = p_event->p_next;

                if (ancs_client.p_app_cb)
                {
                    event_data.notification.p_data = (wiced_bt_ancs_client_notification_data_t *) &p_event->data;
                    (*ancs_client.p_app_cb)(WICED_BT_ANCS_CLIENT_EVENT_NOTIFICATION, &event_data);
                }

                wiced_bt_free_buffer(p_event);

                /* Change current first event from queued event type to client event type. */
                if (ancs_client.p_first_event != NULL)
                {
                    p_queued_event = (ancs_client_queued_event_t *) ancs_client.p_first_event;

                    if ((ancs_client.p_first_event = (ancs_client_event_t *) wiced_bt_get_buffer(sizeof(ancs_client_event_t))) == NULL)
                    {
                        /* This must not be happened since we just free the buffer in the previous step.
                         * If this happens, we shall consider to lock this section to ensure the memory allocation. */
                        ANCS_CLIENT_TRACE("Err: Failed to get buf to copy\n");
                        break;
                    }

                    memset((void *) ancs_client.p_first_event, 0, sizeof(ancs_client_event_t));

                    ancs_client.p_first_event->p_next = p_queued_event->p_next;

                    memcpy((void *) &ancs_client.p_first_event->data.basic,
                           (void *) &p_queued_event->data.basic,
                           sizeof(wiced_bt_ancs_client_notification_data_basic_t));

                    wiced_bt_free_buffer(p_queued_event);
                }
            }
            else
            {
                // start reading attributes for the next message
                status = ancs_client_send_next_get_notification_attributes_command(ancs_client.p_first_event->data.basic.notification_uid);
                if (status == WICED_BT_GATT_BUSY)
                {
                    // another GATT procedure is currently active, retry in a second
                    ANCS_CLIENT_TRACE("busy retrieve:%d\n", ancs_client.p_first_event->data.basic.notification_uid);
                    wiced_start_timer(&ancs_client.ancs_retry_timer, ANCS_CLIENT_GET_NOTIFICATION_ATTRIBUTE_RETRY_TIMEOUT);
                }
                break;
            }
        }
    }
}

/*
 * Process Data Source messages from the phone.
 * This can be new or continuation of the previous message
 */
static void ancs_client_process_data_source(uint8_t *data, int len)
{
//    uint8_t      attr_id;
    uint8_t      attr_len;

//    ANCS_CLIENT_TRACE("Data source left to read:%d len:%d\n", ancs_client.data_left_to_read, len);

    // check if this is a continuation of the previous message
    if (ancs_client.data_left_to_read)
    {
        memcpy(&ancs_client.data_source_buffer[ancs_client.data_source_buffer_offset], data, len);
        ancs_client.data_source_buffer_offset += len;
        ancs_client.data_left_to_read -= len;
        if (ancs_client.data_left_to_read <= 0)
        {
            ancs_client_process_event_attribute(&ancs_client.data_source_buffer[5], ancs_client.data_source_buffer_offset - 5);
        }
    }
    else
    {
        // start of the new message
//        attr_id  = data[5];
        attr_len = data[6] + (data[7] << 8);
        // ANCS_CLIENT_TRACE("ANCS Data Notification Attribute:%04x len %d\n", attr_id, attr_len);
        if (attr_len <= len - 8)
        {
            ancs_client_process_event_attribute(&data[5], len - 5);
        }
        else
        {
            // whole message did not fit into the message, phone should send addition data
            memcpy(&ancs_client.data_source_buffer[0], data, len);
            ancs_client.data_source_buffer_offset = len;
            ancs_client.data_left_to_read = attr_len - len + 8;
        }
    }
}

/**
 * wiced_bt_ancs_client_notification_handler
 *
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint16_t handle = p_data->response_data.att_value.handle;
    uint8_t     *data  = p_data->response_data.att_value.p_data;
    uint16_t len    = p_data->response_data.att_value.len;

    // We can receive notifications on Notification Source or Data Source
    // Phone also can send several notifications on the data source if it did not fit.
    if (ancs_client.data_left_to_read || (handle == ancs_client.data_source_val_hdl))
    {
        ancs_client_process_data_source(data, len);
    }
    else if (handle == ancs_client.notification_source_val_hdl)
    {
        ancs_client_process_notification_source(data, len);
    }
    else
    {
        ANCS_CLIENT_TRACE("ANCS Notification bad handle:%02x, %d\n", (uint16_t )handle, len);
    }
}

/**
 * wiced_bt_ancs_client_indication_handler
 *
 * Process GATT Indications from the client.  Application passes it here only
 * if the handle belongs to this service.
 *
 * @param p_data    : refer to wiced_bt_gatt_operation_complete_t
 */
void wiced_bt_ancs_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
}

static void ancs_client_set_client_config_descriptor(uint16_t conn_id, uint16_t handle, uint16_t value)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 1];
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )buf;
    uint16_t               u16 = value;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = handle;
    p_write->offset   = 0;
    p_write->len      = 2;
    p_write->auth_req = GATT_AUTH_REQ_NONE;
    p_write->value[0] = u16 & 0xff;
    p_write->value[1] = (u16 >> 8) & 0xff;

    // Register with the server to receive notification
    status = wiced_bt_gatt_send_write(conn_id, GATT_WRITE, p_write);

    ANCS_CLIENT_TRACE("wiced_bt_gatt_send_write %d\n", status);

    (void) status;
}

static void ancs_client_send_discover(uint16_t conn_id, wiced_bt_gatt_discovery_type_t type, uint16_t uuid,
        uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_discovery_param_t param = {0};
    wiced_bt_gatt_status_t          status;

    if (uuid != 0)
    {
        param.uuid.len = LEN_UUID_16;
        param.uuid.uu.uuid16 = uuid;
    }

    param.s_handle = s_handle;
    param.e_handle = e_handle;

    status = wiced_bt_gatt_send_discover(conn_id, type, &param);

    ANCS_CLIENT_TRACE("wiced_bt_gatt_send_discover %d\n", status);

    (void) status;
}
