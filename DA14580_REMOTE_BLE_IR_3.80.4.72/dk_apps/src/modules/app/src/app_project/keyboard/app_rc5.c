/**
 ****************************************************************************************
 *
 * @file app_rc5.c
 *
 * @brief RC5 IR protocol module.
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

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>

#include "periph_setup.h"
#include "ir_driver.h"
#include "app_rc5.h"

#define __RETAINED __attribute__((section("retention_mem_area0"), zero_init))

/*
 * RETAINED VARIABLE DECLARATIONS
 ****************************************************************************************
 */
uint8_t ir_rc5_toggle_bit __RETAINED;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief  Initialize IR interface for support RC5 protocol.
 ****************************************************************************************
 */
void app_rc5_init(void)
{
    ir_init();
    ir_set_carrier_freq(RC5_CARRIER_ON_TIME, RC5_CARRIER_OFF_TIME);
    ir_set_logic_one_time(IR_LOGIC_ONE_STARTS_SPACE, RC5_LOGIC_ONE_MARK, RC5_LOGIC_ONE_SPACE);
    ir_set_logic_zero_time(IR_LOGIC_ZERO_STARTS_MARK, RC5_LOGIC_ZERO_MARK, RC5_LOGIC_ZERO_SPACE);
    ir_set_repeat_type(IR_REPEAT_FROM_CODE_FIFO);
    ir_set_repeat_time(RC5_REPEAT_TIME);
}


/**
 ****************************************************************************************
 * @brief  Start sendind specified RC5 command using IR interface.
 *
 * @param[in] command: RC5 command which will be send.
 *
 ****************************************************************************************
 */
void app_rc5_send_command(uint16_t key)
{
    app_force_active_mode();
    ir_flush_code_fifo();
    ir_insert_digital_message(IR_CODE_FIFO, 2, 0x03); //start bits
    ir_insert_digital_message(IR_CODE_FIFO, 1, ir_rc5_toggle_bit); //toggle bit
    ir_insert_digital_message(IR_CODE_FIFO, 5, ir_reverse_bit_order((key >> 8) & 0x00FF, 5)); //address
    ir_insert_digital_message(IR_CODE_FIFO, 6, ir_reverse_bit_order(key & 0x00FF, 6)); //command
    ir_start();
}


/**
 ****************************************************************************************
 * @brief  Stop sending RC5 command.
 ****************************************************************************************
 */
void app_rc5_stop_sending(void)
{
    ir_stop();
    ir_rc5_toggle_bit ^= 0x01;
    //while(ir_is_busy()){};
}


/**
 ****************************************************************************************
 * @brief  Check busy status.
 *
 * @return True if busy or false if not.
 *
 ****************************************************************************************
 */
bool app_rc5_is_busy(void)
{
    return ir_is_busy();
}

/// @} APP
