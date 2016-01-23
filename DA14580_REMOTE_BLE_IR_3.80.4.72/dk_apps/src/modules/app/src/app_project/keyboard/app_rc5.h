/**
 ****************************************************************************************
 *
 * @file app_rc5.h
 *
 * @brief RC5 IR protocol module header file.
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

#ifndef APP_RC5_H_
#define APP_RC5_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RC5
 *
 * @brief RC5 IR protocol module.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer definition
#include <co_bt.h>

 /*
 * DEFINES
 ****************************************************************************************
 */

#define RC5_CARRIER_ON_TIME         120
#define RC5_CARRIER_OFF_TIME        324

#define RC5_LOGIC_ONE_MARK          32
#define RC5_LOGIC_ONE_SPACE         32

#define RC5_LOGIC_ZERO_MARK         32
#define RC5_LOGIC_ZERO_SPACE        32

#define RC5_REPEAT_TIME             4168

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

void app_rc5_init(void);

void app_rc5_send_command(uint16_t key);

void app_rc5_stop_sending(void);

bool app_rc5_is_busy(void);

/// @} APP

#endif // APP_RC5_H_
