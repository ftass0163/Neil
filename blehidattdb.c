/*******************************************************************************
* THIS INFORMATION IS PROPRIETARY TO BROADCOM CORP 
*
* ------------------------------------------------------------------------------
*
* Copyright (c) 2011 Broadcom Corp.
*
*          ALL RIGHTS RESERVED
*
********************************************************************************
*
* File Name: blehidattdb.c
*
* Abstract: This file contains the BLE HID attribute DB
*
* Functions:
*
*******************************************************************************/
#include "bleremote.h"
#include "bleapp.h"
#include "bleprofile.h"
#include "gpiodriver.h"

#ifdef WICED_OTA_FWU
#include "ws_upgrade_ota.h"

// Define macros helpful for GATT DB definitions

#define BIT16_TO_8( val ) \
    (UINT8)(  (val)        & 0xff),/* LSB */ \
    (UINT8)(( (val) >> 8 ) & 0xff) /* MSB */

#define PRIMARY_SERVICE_UUID128(handle, service)  \
    BIT16_TO_8((UINT16)(handle)), \
    LEGATTDB_PERM_READABLE, \
    0x12, 0x00, \
    BIT16_TO_8(UUID_ATTRIBUTE_PRIMARY_SERVICE), \
    service

#define _CHARACTERISTIC_UUID16(handle, handle_value, uuid, properties, permission, value_len) \
        BIT16_TO_8((UINT16)(handle)), \
    LEGATTDB_PERM_READABLE, \
    BIT16_TO_8(0x07), \
    BIT16_TO_8(UUID_ATTRIBUTE_CHARACTERISTIC), \
    (UINT8)(properties), \
    BIT16_TO_8((UINT16)(handle_value)), \
    BIT16_TO_8(uuid), \
    BIT16_TO_8((UINT16)(handle_value)), \
    (UINT8)(permission), \
    BIT16_TO_8((UINT16)(value_len+2)), \
    BIT16_TO_8(uuid)

#define CHARACTERISTIC_UUID128(handle, handle_value, uuid, properties, permission, value_len) \
    BIT16_TO_8((UINT16)(handle)), \
    LEGATTDB_PERM_READABLE, \
    0x15, 0x00, \
    BIT16_TO_8(UUID_ATTRIBUTE_CHARACTERISTIC), \
    (UINT8)(properties), \
    BIT16_TO_8((UINT16)(handle_value)), \
    uuid, \
    BIT16_TO_8((UINT16)(handle_value)), \
    (UINT8)(permission | LEGATTDB_PERM_SERVCIE_UUID_128), \
    BIT16_TO_8((UINT16)(value_len+16)), \
    uuid

#define CHARACTERISTIC_UUID128_WRITABLE(handle, handle_value, uuid, properties, permission, value_len) \
    BIT16_TO_8((UINT16)(handle)), \
    LEGATTDB_PERM_READABLE, \
    0x15,0x00, \
    BIT16_TO_8(UUID_ATTRIBUTE_CHARACTERISTIC), \
    (UINT8)(properties), \
    BIT16_TO_8((UINT16)(handle_value)), \
    uuid, \
    BIT16_TO_8((UINT16)(handle_value)), \
    (UINT8)(permission | LEGATTDB_PERM_SERVCIE_UUID_128), \
    BIT16_TO_8((UINT16)(value_len+16)), \
    BIT16_TO_8((UINT16)(value_len)), \
    uuid

#define CHAR_DESCRIPTOR_UUID16_WRITABLE(handle, uuid, permission, value_len) \
    BIT16_TO_8((UINT16)(handle)), \
    (UINT8)(permission), \
    BIT16_TO_8((UINT16)(value_len+2)), \
    BIT16_TO_8((UINT16)(value_len)), \
    BIT16_TO_8(uuid)
#endif

/*
 * BLE HID Remote app GATT database
*/
PLACE_IN_DROM const UINT8 blehidremote_db_data[]=
{
0x01, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04, 0x00,              // length
        0x00, 0x28,              // PRIMARY SERVICE
    0x01,0x18,                   // Generic Attribute Service
0x02, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x10,0x03,0x00,0x05,0x2a,    // NOTIFY, Handle, Service Changed Characteristic
0x03, 0x00,                      // handle
        LEGATTDB_PERM_NONE,      // permission
        0x06, 0x00,              // length
        0x05, 0x2a,              // Service Changed Characteristicvalue
    0x00,0x00,0x00,0x00,         // Characteristic value
0x14, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04, 0x00,              // length
        0x00, 0x28,              // PRIMARY SERVICE
    0x00,0x18,                   // Generic Access Profile
0x15, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02,0x16,0x00,0x00,0x2a,    // READ, Handle, DEVICE NAME
0x16, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x1A, 0x00,              // length
        0x00, 0x2a,              // DEVICE NAME characteristic value
    'B', 'r', 'o', 'a', 'd', 'c', 'o', 'm', ' ', 'W', 'i', 'r', 'e', 'l', 'e', 's', 's', ' ', 'R', 'e', 'm', 'o', 't', 'e', // Name
0x17, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02,0x18,0x00,0x01,0x2a,    // Notify, Handle, APPEARANCE
0x18, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04, 0x00,            // length
        0x01, 0x2a,              // APPEARANCE characteristic value
    0xC1,0x03,	                 // Keyboard - tentative - TODO:
0x19, 0x00,
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02,0x1A,0x00,0x04,0x2a,    // Notify, Handle, Peripheral Prefered Connection Parameters
0x1A, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x0A, 0x00,            // length
        0x04, 0x2A,               //Peripheral Prefered Connection Parameters
        0x06, 0x00, 0x06, 0x00,   // mininum connection interval, maximum connection interval
    0x14,0x00,0x2C, 0x01,        // slave latency, connection supervision timeout multiplier
0x28, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04, 0x00,             // length
        0x00, 0x28,              // PRIMARY SERVICE 
	0x0A,0x18,                   // Device Information Service
0x29, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,               // length
        0x03, 0x28,              // CHARACTERISTIC
	0x02,0x2a,0x00,0x50,0x2A,    // PnP information
0x2a, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x09, 0x00,              // length
        0x50, 0x2A,              // PnP info
    0x01, 0x0F, 0x00, 0x12,0x34, 0x01, 0x00, // BT SIG, BRCM, 0x3412, 0x0001
0x2b, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02,0x2c,0x00,0x29,0x2a,    // READ, Handle, Manudacturer name
0x2c, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x10, 0x00,              // length
        0x29, 0x2a,              // Manudacturer name characteristic value
    'B', 'r', 'o', 'a', 'd', 'c', 'o', 'm', ' ', 'C', 'o', 'r', 'p', '.',
    _CHARACTERISTIC_UUID16(0x002D,
                          0x002E,
                          0x2A26,
                          LEGATTDB_CHAR_PROP_READ,
                          LEHID_GATTDB_PERM_READABLE,
                          FW_REV_LEN),
                          FW_REV,
0x30, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission  
        0x04, 0x00,              // length
        0x00, 0x28,              // PRIMARY SERVICE
    0x0F, 0x18,                  // Battery service
0x31, 0x00,                      // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02,0x32,0x00,0x19,0x2A,    // Read,handle, Battery level characteristic
0x32,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission  
        0x03, 0x00,              // length
        0x19, 0x2A,              // BATTERY LEVEL
    0x00,                        // Battery level
0x33,0x00, // handle
        LEHID_GATTDB_PERM_READABLE |
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04, 0x00, // length
        0x02, 0x00, // max length
        0x02,0x29, // Client characteristic configuration
    0x00, 0x00, // Notification disable, Indication disable
0x34,0x00, // handle
        LEHID_GATTDB_PERM_READABLE, // permission
        0x04, 0x00, // length
        0x08,0x29, // Report reference
    BATTERY_REPORT_ID ,0x01, // Report ID (battery report), Input report
0x40,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission  
        0x04, 0x00,              // length
        0x00, 0x28,              // PRIMARY SERVICE
    0x13, 0x18,                  // Scan Parameters service
0x41,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x04, 0x42, 0x00, 0x4F, 0x2A,// Write without response, handle, Scan Interval Window
0x42,0x00,                       // handle
        LEHID_GATTDB_PERM_WRITABLE,  // permission
        0x06, 0x00,              // length
        0x04, 0x00,              // max length
        0x4F, 0x2A,              // Scan Interval Window
    0x00, 0x00, 0x00, 0x00,      // vaue
0x4F,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04, 0x00,              // length
        0x00, 0x28,              // PRIMARY SERVICE
    0x12, 0x18,                  // HID service
0x50,0x00, // handle
        LEHID_GATTDB_PERM_READABLE, // permission
        0x08, 0x00, // length
        0x02, 0x28, // Include BAS SERVICE
    0x30, 0x00, 0x34, 0x00, 0x0F, 0x18,// start, end, BAS UUID16   
// HID control point
0x51,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x04,0x52,0x00,0x4C,0x2A,    // WriteWithoutResponse, handle, HID control point
0x52,0x00,                       // handle
        LEHID_GATTDB_PERM_WRITE_CMD, // permission
        0x03, 0x00,              // length
        0x01, 0x00,              // max length
        0x4C,0x2A,               // HID Control point
    0xFF,                        // Value - undefined. Writable only
//HID info.
0x53,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,               // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02, 0x54,0x00,0x4A,0x2A,   // Read, handle, HID information
0x54,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x06,0x00,               // length
        0x4A,0x2A,               // HID information
    0x00, 0x01, 0x00, 0x00,      // Verison 1.00, Not localized, Cannot remote wake, not normally connectable
//Report map
0x55,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,               // length
        0x03, 0x28,              // CHARACTERISTIC
    0x02, 0x56,0x00,0x4B,0x2A,   // Read, handle, Report Map

0x56,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x0C,0x01,                    // length
        0x4B,0x2A,               // Report Map

            // STD_KB_REPORT_ID
            // Input Report, 8 bytes
            // 1st byte:Keyboard LeftControl/Keyboard Right GUI
            // 2nd byte:Constant, 3rd ~ 6th: keycode
            // Output Report, 1 byte: LED control
            0x05 , 0x01,                    // USAGE_PAGE (Generic Desktop)
            0x09 , 0x06,                    // USAGE (Keyboard)
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , STD_KB_REPORT_ID,        //    REPORT_ID (1)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x95 , 0x08,                    //    REPORT_COUNT (8)
            0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
            0x19 , 0xE0,                    //    USAGE_MINIMUM (Keyboard LeftControl)
            0x29 , 0xE7,                    //    USAGE_MAXIMUM (Keyboard Right GUI)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
            0x81 , 0x02,                    //    INPUT (Data,Var,Abs)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
            0x95 , 0x05,                    //    REPORT_COUNT (5)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x05 , 0x08,                    //    USAGE_PAGE (LEDs)
            0x19 , 0x01,                    //    USAGE_MINIMUM (Num Lock)
            0x29 , 0x05,                    //    USAGE_MAXIMUM (Kana)
            0x91 , 0x02,                    //    OUTPUT (Data,Var,Abs)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x03,                    //    REPORT_SIZE (3)
            0x91 , 0x03,                    //    OUTPUT (Cnst,Var,Abs)
            0x95 , 0x06,                    //    REPORT_COUNT (6)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
            0x19 , 0x00,                    //    USAGE_MINIMUM (Reserved (no event indicated))
            0x29 , 0xFF,                    //    USAGE_MAXIMUM (Reserved (no event indicated))
            0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
            0xC0,                           // END_COLLECTION

            //Bit mapped report, BITMAPPED_REPORT_ID
            0x05 , 0x0C,                    // USAGE_PAGE (Consumer Devices)
            0x09 , 0x01,                    // USAGE (Consumer Control)
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , BITMAPPED_REPORT_ID,     //    REPORT_ID (2)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x95 , 0x12,                    //    REPORT_COUNT (18)
            //byte 0
            0x0A , 0x94 , 0x01,             //    USAGE (AL Local Machine Browser)
            0x0A , 0x92 , 0x01,             //    USAGE (AL Calculator)
            0x0A , 0x83 , 0x01,             //    USAGE (Media)
            0x0A , 0x23 , 0x02,             //    USAGE (WWW Home)
            0x0A , 0x8A , 0x01,             //    USAGE (AL Email)
            0x0A , 0x82 , 0x01,             //    USAGE (Programmable Button Control)
            0x0A , 0x21 , 0x02,             //    USAGE (AC Search)
            0x0A , 0x24 , 0x02,             //    USAGE (AC Back)
            // byte 1
            0x0A , 0x25 , 0x02,             //    USAGE (AC Forward)
            0x0A , 0x26 , 0x02,             //    USAGE (AC Stop)
            0x0A , 0x27 , 0x02,             //    USAGE (AC Refresh)
            0x09 , 0xB6,                    //    USAGE (Scan Previous Track)
            0x09 , 0xB5,                    //    USAGE (Scan Next Track)
            0x09 , 0xB7,                    //    USAGE (Stop)
            0x09 , 0xB0,                    //    USAGE (Play)
            0x09 , 0xE9,                    //    USAGE (Volume Up)
            //byte 2
            0x09 , 0xEA,                    //    USAGE (Volume Down)
            0x09 , 0xE2,                    //    USAGE (Mute)
            0x81 , 0x02,                    //    INPUT (Data,Var,Abs)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x06,                    //    REPORT_SIZE (6)
            0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
            0xC0,                           // END_COLLECTION

            //motion report,  MOTION_REPORT_ID
            0x05 , 0x0C,                    // Usage Page (Consumer Devices)
            0x09 , 0x01,                    //    Usage (Consumer Control)
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , 0x08,                    //    REPORT_ID (0x08)
            0x95 , 0x12,                     //    REPORT_COUNT (18)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
            0xC0,                           // END_COLLECTION

            //User defined 0 report, 0x0A 
            0x05 , 0x0C,                    // Usage Page (Consumer Devices)
            0x09 , 0x01,                    //    Usage (Consumer Control)
            0xA1 , 0x01,                    //    Collection (Application)
            0x85 , 0x0A,                    //      Report ID=0A
            0x95 , 0x08,                    //    REPORT_COUNT (8)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
            0x19 , 0x00,                    //    USAGE_MINIMUM (Reserved (no event indicated))
            0x29 , 0xFF,                    //    USAGE_MAXIMUM (Reserved (no event indicated))
            0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
            0xC0,                           // END_COLLECTION

            //audio report, AUDIO_REPORT_ID 
            0x05 , 0x0C,                    // Usage Page (Consumer Devices)
            0x09 , 0x01,                    //    Usage (Consumer Control)
            0xA1 , 0x01,                    //    Collection (Application)
            0x85 , AUDIO_REPORT_ID,         //    Report ID=0xF7
#ifndef ATT_MTU_SIZE_64
            0x95 , 0x14,                     //    REPORT_COUNT (20)
#else
            0x95 , 0x3A,                     //    REPORT_COUNT (58)
#endif            
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
            0xC0,                           // END_COLLECTION

            //voice ctrl report, VOICE_CTL_REPORT_ID 
            0x05 , 0x0C,                    // Usage Page (Consumer Devices)
            0x09 , 0x01,                    //    Usage (Consumer Control)
            0xA1 , 0x01,                    //    Collection (Application)
            0x85 , 0xF8,                    //    Report ID=0xF8
            0x95 , 0x0B,                     //    REPORT_COUNT (11)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
            0x95 , 0x0B,                    //    REPORT_COUNT (11)
            0x75 , 0x01,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0xB1 , 0x00,                    //    FEATURE (Data,Ary,Abs)            
            0xC0,                           // END_COLLECTION

            //Battery report
            0x05 , 0x0C,                    // Usage Page (Consumer Devices),
            0x09 , 0x01,                    // Usage (Consumer Control),
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , BATTERY_REPORT_ID,       //    REPORT_ID (3)
            0x05 , 0x01,                    //    Usage Page (Generic Desktop),
            0x09 , 0x06,                    //    Usage (Keyboard)
            0xA1 , 0x02,                    //    Collection: (Logical),
            0x05 , 0x06,                    //        USAGE PAGE (Generic Device Control),
            0x09 , 0x20,                    //        USAGE (Battery Strength),
            0x15 , 0x00,                    //        Log Min (0),
            0x26 , 0x64 , 0x00,             //        Log Max (255),
            0x75 , 0x08,                    //        Report Size (8),
            0x95 , 0x01,                    //        Report Count (1),
            0x81 , 0x02,                    //        Input (Data, Variable, Absolute),
            0xC0,                           //    END_COLLECTION (Logical)
            0xC0,                           // END_COLLECTION
// include Battery Service
0x57,0x00,                    // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04, 0x00,              // length
        0x07, 0x29,              // External report reference
    0x19,0x2A,                   // Battery level characterisc

// STD Input report
0x5D,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x12,0x5E,0x00,0x4D,0x2A,    // Read|Notify, handle, Report
0x5E,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x0A,0x00,               // length
        0x4D,0x2A,               // Report
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,      // Value. 8 bytes
0x5F,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE |
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04,0x00,               // length
        0x02,0x00,               // max length
        0x02,0x29,               // Client characteristic configuration
    0x00, 0x00,                  // Notification disable, Indication disable
0x60,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    STD_KB_REPORT_ID,0x01,                   // Report ID (std report), Input report

// Output report
0x61,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x0E,0x62,0x00,0x4D,0x2A,    // Read|Write|WriteWithoutResponse, handle, Report
0x62,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE |
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x03, 0x00,              // length
        0x01, 0x00,              // max length
        0x4D,0x2A,               // Report
    0x00,                        // Value.
0x63,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    STD_KB_REPORT_ID,0x02,       // Report ID (LED report), Output report

// Bit mapped report, Report ID=2
0x64,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x12,0x65,0x00,0x4D,0x2A,    // Read|Notify, handle, Report
0x65,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x05,0x00,                    // length
        0x4D,0x2A,               // Report
    0x00, 0x00, 0x00,            // Value. 3 bytes
0x66,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE | 
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04,0x00,               // length
        0x02,0x00,               // max length
        0x02,0x29,               // Client characteristic configuration
    0x00, 0x00,                  // Notification disable, Indication disable
0x67,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    BITMAPPED_REPORT_ID,0x01,    // Report ID (bit mapped report), Input report

// motion report
0x68,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,               // length
        0x03, 0x28,              // CHARACTERISTIC
    0x12,0x69,0x00,0x4D,0x2A,    // Read|Notify, handle, Report
0x69,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x014,0x00,               // length
        0x4D,0x2A,               // Report
    0x00,0x00,0x00,0x00,          // Value.
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,   
0x6A,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE |
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04,0x00,               // length
        0x02,0x00,               // max length
        0x02,0x29,               // Client characteristic configuration
    0x00, 0x00,                  // Notification disable, Indication disable
0x6B,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    0x08,0x01,        // Report ID (0x08), Input report

// user defined 0 report
0x6C,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x12,0x6D,0x00,0x4D,0x2A,    // Read|Notify, handle, Report
0x6D,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x0A,0x00,               // length
        0x4D,0x2A,               // Report
    0x00,0x00,0x00,0x00,         // Value.
    0x00,0x00,0x00,0x00,                       
0x6E,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE | 
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04,0x00,               // length
        0x02,0x00,               // max length
        0x02,0x29,               // Client characteristic configuration
    0x00, 0x00,                  // Notification disable, Indication disable
0x6F,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
        0x0A,0x01,               // Report ID (0x0A), Input report

//audio report
0x70,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x12,0x71,0x00,0x4D,0x2A,    // Read|Notify, handle, Report

#ifndef ATT_MTU_SIZE_64
0x71,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x16,0x00,               // length = 20+2
        0x4D,0x2A,               // Report
    0x00,0x00,0x00,0x00,          // Value.
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,    
#else
0x71,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x3C,0x00,               // length = 58+2
        0x4D,0x2A,               // Report
    0x00,0x00,0x00,0x00,          // Value.
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,    
    0x00,0x00,0x00,0x00,          
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00, 
    0x00,0x00,0x00,0x00,          
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,
    0x00,0x00,
#endif    

0x72,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE | 
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04,0x00,              // length
        0x02,0x00,              // max length
        0x02,0x29,               // Client characteristic configuration
    0x00, 0x00,                  // Notification disable, Indication disable
0x73,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    AUDIO_REPORT_ID,0x01,       // Report ID (AUDIO_REPORT_ID), Input report


//voice ctrl report
0x74,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07,0x00,               // length
        0x03, 0x28,              // CHARACTERISTIC
    0x12,0x75,0x00,0x4D,0x2A,    // Read|Notify, handle, Report
0x75,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x0D,0x00,               // length
        0x4D,0x2A,               // Report
    0x00,0x00,0x00,0x00,          // Value.
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,
0x76,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE | 
        LEHID_GATTDB_PERM_WRITABLE, // permission
        0x04,0x00,               // length
        0x02,0x00,               // max length
        0x02,0x29,               // Client characteristic configuration
    0x00, 0x00,                  // Notification disable, Indication disable
0x77,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    0xF8,0x01,       // Report ID (0xF8), Input report

// Feature report
0x78,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x0A,0x79,0x00,0x4D,0x2A,    // Read|Write, handle, Report
0x79,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE |
        LEHID_GATTDB_PERM_WRITABLE |
        LEGATTDB_PERM_VARIABLE_LENGTH, // permission
        0x0D, 0x00,              // length
        0x0B, 0x00,              // max length
        0x4D,0x2A,               // Report
    0x00,0x00,0x00,0x00,          // Value.
    0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,
0x7A,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    0xF8,0x03,       // Report ID (0xF8), Feature report

// Connection control feature
0x7B,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x07, 0x00,               // length
        0x03, 0x28,              // CHARACTERISTIC
    0x0A,0x7C,0x00,0x4D,0x2A,    // Read|Write, handle, Report
0x7C,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE |
        LEHID_GATTDB_PERM_WRITE_REQ, // permission
        0x03, 0x00,              // length
        0x01, 0x00,              // max length
        0x4D,0x2A,               // Report
    0x00,                        // Value.
0x7D,0x00,                       // handle
        LEHID_GATTDB_PERM_READABLE,  // permission
        0x04,0x00,               // length
        0x08,0x29,               // Report reference
    0xCC,0x03,                   // Report ID (Connection control), Feature report

#ifdef SUPPORTING_FINDME
0x90, 0x00,                      // handle
        LEGATTDB_PERM_READABLE,  // permission  
        0x04, 0x00,              // length
        0x00, 0x28,              // PRIMARY SERVICE
    0x02, 0x18,                  // immediate alert
0x91, 0x00,                      // handle
        LEGATTDB_PERM_READABLE,  // permission
        0x07, 0x00,              // length
        0x03, 0x28,              // CHARACTERISTIC
    0x04,0x92,0x00,0x06,0x2A,    // Write No Response, handle, alert level characteristic
0x92,0x00,                       // handle
        LEGATTDB_PERM_WRITE_CMD,  // permission  
        0x03, 0x00,              // length
        0x01, 0x00,              // value len
        0x06, 0x2A,              // ALERT LEVEL
        0x00,                    // Alert Level
    
#endif

#ifdef WICED_OTA_FWU
    // Handle 0xff00: Broadcom vendor specific WICED Smart Upgrade Service.
    // The service has 2 characteristics.  The first is the control point.  Client
    // sends commands, and sensor sends status notifications. Note that
    // UUID of the vendor specific service is 16 bytes, unlike standard Bluetooth
    // UUIDs which are 2 bytes.  _UUID128 version of the macro should be used.
    PRIMARY_SERVICE_UUID128 (HANDLE_WS_UPGRADE_SERVICE, UUID_WS_UPGRADE_SERVICE),
    
    // Handle 0xff01: characteristic WS Control Point, handle 0xff02 characteristic value.
    // This characteristic can be used by the client to send commands to this device
    // and to send status notifications back to the client.  Client has to enable
    // notifications by updating Characteristic Client Configuration Descriptor
    // (see handle ff03 below).  Note that UUID of the vendor specific characteristic is
    // 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
    // of the macro should be used.  Also note that characteristic has to be _WRITABLE
    // to correctly enable writes from the client.
    CHARACTERISTIC_UUID128_WRITABLE (HANDLE_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
                                     HANDLE_WS_UPGRADE_CONTROL_POINT, UUID_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
                                     LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
                                     LEGATTDB_PERM_WRITE_REQ, 5),
        0x00,0x00,0x00,0x00,0x00,
    
    // Handle 0xff03: Characteristic Client Configuration Descriptor.
    // This is a standard GATT characteristic descriptor.  2 byte value 0 means that
    // message to the client is disabled.  Peer can write value 1 to enable
    // notifications or respectively.  Note _WRITABLE in the macro.  This
    // means that attribute can be written by the peer.
    CHAR_DESCRIPTOR_UUID16_WRITABLE (HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
                                     UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,
    
    // Handle 0xff04: characteristic WS Data, handle 0xff05 characteristic value
    // This characteristic is used to send next portion of the FW.  Similar to the
    // control point, characteristic should be _WRITABLE and 128bit version of UUID is used.
    CHARACTERISTIC_UUID128_WRITABLE (HANDLE_WS_UPGRADE_CHARACTERISTIC_DATA,
                                     HANDLE_WS_UPGRADE_DATA, UUID_WS_UPGRADE_CHARACTERISTIC_DATA,
                                     LEGATTDB_CHAR_PROP_WRITE,
                                     LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ,  20),
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,
#endif

};

const UINT16 blehidremote_db_size = sizeof(blehidremote_db_data);

PLACE_IN_DROM const BLE_PROFILE_CFG blehidremote_cfg =
{
    0, // UINT16 fine_timer_interval; //ms
    4, // HIGH_UNDIRECTED_DISCOVERABLE, // UINT8 default_adv; //default adv
    0, // UINT8 button_adv_toggle; //pairing button make adv toggle (if 1) or always on (if 0)    
    60, //UINT16 high_undirect_adv_interval; //slots
    48, //UINT16 low_undirect_adv_interval; //slots
    600, // UINT16 high_undirect_adv_duration; //seconds 
    180, // UINT16 low_undirected_adv_duration; //seconds    
    30, //UINT16 high_direct_adv_interval; //seconds
    60,  //UINT16 low_direct_adv_interval; //seconds
    2,  // UINT16 high_direct_adv_duration; //seconds
    5, //  UINT16 low_direct_adv_duration; //seconds    
    "AMPAK RC", //char local_name[LOCAL_NAME_LEN_MAX];
    0x20, 0x25,  0x8c,     //char cod[COD_LEN];
    "1.00", //char ver[VERSION_LEN];
#ifdef __BLEHID_SECURITY__
    SECURITY_REQUEST,// UINT8 encr_required; // if 1, encryption is needed before sending indication/notification
#else
    0, // UINT8 encr_required; // if 1, encryption is needed before sending indication/notification
#endif
    0, // UINT8 disc_required;// if 1, disconnection after confirmation
    1, //UINT8 test_enable;   //TEST MODE is enabled when 1
    0x04,  //  UINT8 tx_power_level; //dbm
    30,  // UINT8 con_idle_timeout; //second   0-> no timeout
    5, //    UINT8 powersave_timeout; //second  0-> no timeout    
    {0x0032, // battery level handle
     0x0052, // HID protocol mode
#ifdef SUPPORTING_FINDME
     0x0092,  // alert level
#else
     0x00,
#endif
      0x00,
      0x00}, // UINT16 hdl[HANDLE_NUM_MAX];   //GATT HANDLE number
    {UUID_SERVICE_BATTERY_TEMP, 
     UUID_SERVICE_HID, 
#ifdef SUPPORTING_FINDME      
     UUID_SERVICE_IMMEDIATE_ALERT,
#else
     0x00,
#endif
      0x00,
      0x00}, // UINT16 serv[HANDLE_NUM_MAX];  //GATT service UUID
    {UUID_CHARACTERISTIC_BATTERY_LEVEL,
     0x2A4E,
#ifdef SUPPORTING_FINDME      
     UUID_CHARACTERISTIC_ALERT_LEVEL,
#else
     0x00,
#endif
      0x00,
      0x00}, // UINT16 cha[HANDLE_NUM_MAX];   // GATT characteristic UUID   
    1, // UINT8 findme_locator_enable; //if 1 Find me locator is enable
    10, // UINT8 findme_alert_level; //alert level of find me
    0, // UINT8 client_grouptype_enable; // if 1 grouptype read can be used
    0, // UINT8 linkloss_button_enable; //if 1 linkloss button is enable      
    0, // UINT8 pathloss_check_interval; //second        
    5, //UINT8 alert_interval; //interval of alert
    10, //UINT8 high_alert_num;     //number of alert for each interval
    5, //UINT8 mild_alert_num;     //number of alert for each interval
    1, // UINT8 status_led_enable;    //if 1 status LED is enable
    2, //UINT8 status_led_interval; //second
    5, // UINT8 status_led_con_blink; //blink num of connection   
    3, // UINT8 status_led_dir_adv_blink; //blink num of dir adv
    3, //UINT8 status_led_un_adv_blink; //blink num of undir adv
    15, // UINT16 led_on_ms;  //led blink on duration in ms
    20, // UINT16 led_off_ms; //led blink off duration in ms
    0, // UINT16 buz_on_ms; //buzzer on duration in ms
    0, // UINT8 button_power_timeout; // seconds
    0, // UINT8 button_client_timeout; // seconds
    1, //UINT8 button_discover_timeout; // seconds
    10, //UINT8 button_filter_timeout; // seconds
#ifdef BLE_UART_LOOPBACK_TRACE
    15, //UINT8 button_uart_timeout; // seconds 
#endif        
}; 

PLACE_IN_DROM const BLE_PROFILE_PUART_CFG blehidremote_puart_cfg =
{
    115200,          // UINT32 baudrate; 
    32,              // UINT8  txpin; //GPIO pin number
    PUARTDISABLE|33, // UINT8  rxpin; //GPIO pin number
};

#ifdef SUPPORTING_FINDME
PLACE_IN_DROM const BLE_PROFILE_GPIO_CFG blehidremote_gpio_cfg =
{
    {BLEREMOTE_PWM_BUZ, BLEREMOTE_GPIO_LED, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,-1}, // UINT8 gpio_pin[GPIO_NUM_MAX];  //pin number of gpio
    {BLEREMOTE_PWM_BUZ_CONFIG,
     BLEREMOTE_GPIO_LED_CONFIG,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0}, // UINT16 gpio_flag[GPIO_NUM_MAX]; //flag of gpio
};
#else
PLACE_IN_DROM const BLE_PROFILE_GPIO_CFG blehidremote_gpio_cfg =
{
    {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,-1}, // UINT8 gpio_pin[GPIO_NUM_MAX];  //pin number of gpio
    {0,
     0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0}, // UINT16 gpio_flag[GPIO_NUM_MAX]; //flag of gpio
};
#endif
