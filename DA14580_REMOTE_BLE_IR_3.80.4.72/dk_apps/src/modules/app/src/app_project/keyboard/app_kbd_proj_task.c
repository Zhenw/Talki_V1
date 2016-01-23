/**
 ****************************************************************************************
 *
 * @file app_kbd_proj_task.c
 *
 * @brief HID Keyboard handlers.
 *
 * Copyright (C) 2013. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"               // SW configuration

#if (BLE_APP_PRESENT)

#include "arch.h"                      // Platform Definitions
#include "ke_task.h"
#include "co_error.h"                  // Error Codes Definition

#include "app.h"                       // Application Definition
#include "app_task.h"                  // Application Task API
#include "gapc_task.h"                 // GAP Controller Task API
#include "gapm_task.h"                 // GAP Manager Task API
#include "gattc_task.h"                // GATT Controller Task API
#include "gap.h"                       // GAP Definitions

#include "app_kbd.h"
#include "app_kbd_key_matrix.h"
#include "app_kbd_debug.h"
#include "hogpd_task.h"                // HID over GATT


int extended_timer_cnt __attribute__((section("retention_mem_area0"), zero_init));

// Report Descriptor == Report Map (HID1_11.pdf section E.6)
KBD_TYPE_QUALIFIER uint8 report_map[] KBD_ARRAY_ATTRIBUTE =
	{
		0x05, 0x01,         // Usage Page (Generic Desktop)
		0x09, 0x06,         // Usage (Keyboard)
		0xA1, 0x01,         // Collection: (Application)
        0x85, 0x01,         //  Report ID (1)
		0x05, 0x07,         //  Usage Page (Key Codes)
		0x19, 0xE0,         //  Usage Minimum (224)
		0x29, 0xE7,         //  Usage Maximum (231)
		0x15, 0x00,         //  Logical Minimum (0)
		0x25, 0x01,         //  Logical Maximum (1)
		0x75, 0x01,         //  Report Size (1)
		0x95, 0x08,         //  Report Count (8)
		0x81, 0x02,         //  Input: (Data, Variable, Absolute) ; Modifier byte
		0x95, 0x01,         //  Report Count (1)
		0x75, 0x08,         //  Report Size (8)
		0x81, 0x01,         //  Input: (Constant) ; Reserved byte
		0x95, 0x05,         //  Report Count (5)
		0x75, 0x01,         //  Report Size (1)
		0x05, 0x08,         //  Usage Page (LEDs)
		0x19, 0x01,         //  Usage Minimum (1)
		0x29, 0x05,         //  Usage Maximum (5)
		0x91, 0x02,         //  Output: (Data, Variable, Absolute) ; LED report
		0x95, 0x01,         //  Report Count (1)
		0x75, 0x03,         //  Report Size (3)
		0x91, 0x01,         //  Output: (Constant); LED report padding
		0x95, 0x06,         //  Report Count (6)
		0x75, 0x08,         //  Report Size (8)
		0x15, 0x00,         //  Log Minimum (0)
		0x25, 0x65,         //  Log Maximum (101)
		0x05, 0x07,         //  Usage Page (Key Codes)
		0x19, 0x00,         //  Usage Minimum (0)
		0x29, 0x65,         //  Usage Maximum (101)
		0x81, 0x00,         //  Input: (Data, Array) ; Key arrays (6 bytes)
		0xC0,               // End Collection
        0x05, 0x0C,         // Usage Page (Consumer Devices)
        0x09, 0x01,         // Usage (Consumer Control)
        0xA1, 0x01,         // Collection (Application)
        
        
        0x85, 0x03,         //  Report ID (3)
        0x15, 0x00,         //  Logical Minimum (0)
        0x25, 0x01,         //  Logical Maximum (1)
//byte[0].0 .. byte[1].1       
		0x95, 0x0A,         //g  Report Count (10)
		0x75, 0x01,         //g  Report Size (1)
        0x09, 0x30,         //  Usage (Power) (OOC)
		0x09, 0xB0,			//  Usage (Play) (OOC)
		0x09, 0xB1,			//  Usage (Pause) (OOC)
		0x09, 0xB2,			//  Usage (Record) (OOC)
		0x09, 0xB3,			//  Usage (Fast Forward) (OOC)
 		0x09, 0xB4,			//  Usage (Rewind) (OOC)
 		0x09, 0xB7,			//  Usage (Stop) (OOC)
 		0x09, 0xE2,			//  Usage (Mute) (OOC)
		0x09, 0x40,			//  Usage (Menu) (OOC)
		0x09, 0x41,			//  Usage (Menu Pick) (OOC)
        0x81, 0x22,         //  Input (Data,Var,Abs,NPref)
//byte[1].2 .. byte[1].3	
		0x95, 0x02,         //g  Report Count (2)
        0x09, 0xE9,         //  Usage (Volume Increment) (RTC)
        0x09, 0xEA,         //  Usage (Volume Decrement) (RTC)
        0x81, 0x02,         //  Input (Data,Var,Abs,Pref)
//byte[1].4 .. byte[1].6
		0x95, 0x03,         //g  Report Count (3)
        0x09, 0x9C,         //  Usage (Channel Increment) (OSC)
        0x09, 0x9D,         //  Usage (Channel Decrement) (OSC)
        0x09, 0x20,         //  Usage (+10) (OSC)
        0x81, 0x06,         //  Input (Data,Var,Rel,Pref)
//byte[1].7
		0x95, 0x01,         //g  Report Count (1)
		0x09, 0x61,			//  Usage (Closed Caption) (OOC)
        0x81, 0x06,         //  Input (Data,Var,Rel,Pref)
//byte[2].0 .. byte[2].1
        0x15, 0xFF,         //g  Logical Minimum (-1)
        0x25, 0x01,         //g  Logical Maximum (1)
		0x95, 0x01,         //g  Report Count (1)
        0x75, 0x02,         //g  Report Size (2)
        0x09, 0x60,         //  Usage (Data On Screen) (OOC)
        0x81, 0x26,         //  Input (Data,Var,Rel,NPref)        
//byte[2].2
		0x15, 0x00,         //g  Logical Minimum (0)
		0x25, 0x01,         //g  Logical Maximum (1)
        0x75, 0x01,         //g  Report Size (1)
	    0x09, 0x89,         //  Usage (Media Select TV) (Sel)
        0x81, 0x26,         //  Input (Data,Var,Rel,NPref)
//byte[2].3
	    0x09, 0x63,         //  Usage (VCR/TV) (OSC)
        0x81, 0x06,         //  Input (Data,Var,Rel,Pref)

		0x95, 0x04,         //g  Report Count (4)
		0x81, 0x01,         //  Input: (Constant) ; Reserved byte

        0xC0                // End Collection      
	};

#define REPORT_MAP_LEN sizeof(report_map)


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

 /**
 ****************************************************************************************
 * @brief   Sets the HID Report Map value
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 * @remarks Called when the DB has been initialized.
 *****************************************************************************************
 */
int keyboard_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct hogpd_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
	struct hogpd_set_report_map_req * req = KE_MSG_ALLOC_DYN(HOGPD_SET_REPORT_MAP_REQ, TASK_HOGPD, TASK_APP, hogpd_set_report_map_req, REPORT_MAP_LEN);
	req->report_map_len = REPORT_MAP_LEN;
	req->hids_nb = 0;
	memcpy(&(req->report_map[0]), report_map, REPORT_MAP_LEN);
	ke_msg_send(req);	
	
    // Inform the Application Manager
    struct app_module_init_cmp_evt *cfm = KE_MSG_ALLOC(APP_MODULE_INIT_CMP_EVT,
                                                       TASK_APP, TASK_APP,
                                                       app_module_init_cmp_evt);

    cfm->status = param->status;

    ke_msg_send(cfm);
    
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Called when the HID report has been ACK'ed from the master
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 *****************************************************************************************
 */
int keyboard_ntf_sent_cfm_handler(ke_msg_id_t const msgid,
                                      struct hogpd_ntf_sent_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // Note: PRF_ERR_NTF_DISABLED (0x8A) may arrive if we tried to send a Boot report and the notifications have been disabled for some reason
    
    //Clear pending ack's for param->report_nb == 0 (normal key report) and == 2 (ext. key report)
    if (param->status == PRF_ERR_OK)
    {
        if (param->hids_nb == 0) 
        {
            if (param->report_nb == 0) 
            {
            } 
            else if (param->report_nb == 2) 
            {
            }
        }
    }
            
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Protocol Mode has been updated
 *          Called when the Protocol Mode char is written by the Host
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 *****************************************************************************************
 */
int keyboard_proto_mode_ind_handler(ke_msg_id_t const msgid,
                                      struct hogpd_proto_mode_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if (HAS_HOGPD_BOOT_PROTO)
    {
        kbd_proto_mode = param->proto_mode;

        if ( ((bond_info.info & PROTOCOL_MODE_MASK) >> PROTOCOL_MODE_POS) != param->proto_mode)
        {
            app_alt_pair_store_bond_data();
        }
    }
    else if (param->proto_mode == HOGP_BOOT_PROTOCOL_MODE)
        ASSERT_WARNING(0);
    
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Called when the CCC of a Report is written by the Host
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 *****************************************************************************************
 */
int keyboard_ntf_config_ind_handler(ke_msg_id_t const msgid,
                                      struct hogpd_ntf_cfg_ind const *ind,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    int mask = -1;
    int pos;

    switch(ind->cfg_code)
    {
        case HOGPD_BOOT_KB_IN_REPORT_CFG:
            mask = HID_BOOT_CCC_MASK;
            pos = HID_BOOT_CCC_POS;
            break;
        case HOGPD_REPORT_CFG:
            if (ind->report_nb == 0)
            {
                mask = HID_REPORT_0_CCC_MASK;
                pos = HID_REPORT_0_CCC_POS;
            }
            else
            {
                mask = HID_REPORT_2_CCC_MASK;
                pos = HID_REPORT_2_CCC_POS;
            }
            break;
        default:
            break;
    }
    
    if (mask != -1)
        if ( ((bond_info.info & mask) >> pos) != ind->ntf_en)
            app_alt_pair_store_bond_data();
    
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Called when the Control Point char is written by the Host
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 *****************************************************************************************
 */
int keyboard_ctnl_pt_ind_handler(ke_msg_id_t const msgid,
                                      struct hogpd_ctnl_pt_ind const *ind,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if ( ((bond_info.info & HID_CTNL_PT_CCC_MASK) >> HID_CTNL_PT_CCC_POS) != ind->hid_ctnl_pt)
    {
        app_alt_pair_store_bond_data();
    }
    
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Called when the HID profile is being disabled
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 *****************************************************************************************
 */
int keyboard_disable_ind_handler(ke_msg_id_t const msgid,
                                    struct hogpd_disable_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    if (ke_state_get(dest_id) == APP_CONNECTED)
    {
        // Go to the idle state
        ke_state_set(dest_id, APP_CONNECTABLE);
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Called when the Service Changed indication has been successfully received by the Host
 *          
 * @param[in] msgid 
 * @param[in] param
 * @param[in] dest_id
 * @param[in] src_id
 *
 * @return  KE_MSG_CONSUMED
 *
 *****************************************************************************************
 */
int keyboard_svc_chng_cmp_evt_handler(ke_msg_id_t const msgid,
                                      struct gattc_cmp_evt const *ind,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    if(ind->req_type == GATTC_SVC_CHANGED)
    {
        // Clear bond_info.info so that we do not re-send the Service Changed Indication to this Host again
        bond_info.info &= 0xFF;

        // Update EEPROM
        app_alt_pair_store_bond_data();
    }
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief   Set a timer that supports timeout greater than 5 min
 *          Called when a timeout of more than 5 min (that stack supports) is needed.
 *          Only one such timer (only APP_HID_TIMER) can be on at a time!
 *          
 * @param[in] timer_id 
 * @param[in] task_id
 * @param[in] delay
 *
 * @return  void
 *
 * #remarks if delay is <= KE_TIMER_DELAY_MAX then the timer is "resumed". Thus, if
 *          extended_timer_cnt != 0, it continues counting 
 *          (extended_timer_cnt * KE_TIMER_DELAY_MAX) + delay
 *
 *****************************************************************************************
 */
void app_extended_timer_set(ke_msg_id_t const timer_id, ke_task_id_t const task_id, uint32_t delay)
{
    ASSERT_WARNING(timer_id == APP_HID_INACTIVITY_TIMER);
    
    if (HAS_EXTENDED_TIMERS)
    {        
        if (delay <= KE_TIMER_DELAY_MAX)
            app_timer_set(timer_id, task_id, (uint16_t)delay);
        else
        {
            int rem = delay % KE_TIMER_DELAY_MAX;
            
            extended_timer_cnt = delay / KE_TIMER_DELAY_MAX;
            
            if (rem == 0)
            {
                rem = KE_TIMER_DELAY_MAX;
                extended_timer_cnt--;
            }
            app_timer_set(timer_id, task_id, rem);
        }
    }
    else
    {
        ASSERT_WARNING(delay <= KE_TIMER_DELAY_MAX); 
    
        app_timer_set(timer_id, task_id, (uint16_t)delay);
    }
}


/**
 ****************************************************************************************
 * @brief   Clears a timer that was set with app_extended_timer_set()
 *          Called when a timeout of more than 5 min (that stack supports) was set.
 *          Do not directly use ke_timer_clear on such a timer!
 *          
 * @param[in] timer_id 
 * @param[in] task_id
 *
 * @return  void
 *
 *****************************************************************************************
 */
void app_extended_timer_clear(ke_msg_id_t const timer_id, ke_task_id_t const task_id)
{
    ASSERT_WARNING(timer_id == APP_HID_TIMER);
    
    ke_timer_clear(timer_id, task_id);
    
    if (HAS_EXTENDED_TIMERS)
    {
        extended_timer_cnt = 0;
    }
}

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
