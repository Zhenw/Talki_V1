/**
 ****************************************************************************************
 *
 * Copyright (C) 2014. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */


#ifndef _APP_KBD_MATRIX_SETUP_8001_H_
#define _APP_KBD_MATRIX_SETUP_8001_H_

#include <stdint.h>
#include <stddef.h>

#include "app_kbd_config.h"
#include "app_kbd_macros.h"
#include "da14580_config.h"


/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup HID
 *
 * @brief HID (Keyboard) Application matrix of setup #8.
 *
 * @{
 ****************************************************************************************
 */

#if (HAS_EEPROM)
# define PAIR               (0xF4F1)
# define CLRP               (0xF4F2)
#else
# define PAIR               (0x0000)
# define CLRP               (0x0000)
#endif

#if (HAS_KEYBOARD_MEASURE_EXT_SLP)
# define KB_EXTSLP          (0xF4F3)
#else
# define KB_EXTSLP          (0)
#endif


#define KBD_NR_INPUTS		(8)
#define KBD_NR_OUTPUTS		(4)


#define COLUMN_0_PORT		(2)	//port: 2, pin: 2
#define COLUMN_0_PIN		(2)

#define COLUMN_1_PORT		(3)	//port: 3, pin: 3
#define COLUMN_1_PIN		(3)

#define COLUMN_2_PORT		(2)	//port: 2, pin: 1
#define COLUMN_2_PIN		(1)

#define COLUMN_3_PORT		(1)	//port: 1, pin: 1
#define COLUMN_3_PIN		(1)

#define COLUMN_4_PORT		(1)	//port: 1, pin: 0
#define COLUMN_4_PIN		(0)

#define COLUMN_5_PORT		(3)	//port: 3, pin: 5
#define COLUMN_5_PIN		(5)

#define COLUMN_6_PORT		(2)	//port: 2, pin: 3
#define COLUMN_6_PIN		(3)

#define COLUMN_7_PORT		(2)	//port: 2, pin: 4
#define COLUMN_7_PIN		(4)


// used for cycle optimization
#define P0_HAS_INPUT		(0)
#define P1_HAS_INPUT		(1)
#define P2_HAS_INPUT		(1)
#define P3_HAS_INPUT		(1)

#define ROW_0_PORT			(3)	//port: 3, pin: 2
#define ROW_0_PIN			(2)

#define ROW_1_PORT			(3)	//port: 3, pin: 4
#define ROW_1_PIN			(4)

#define ROW_2_PORT			(1)	//port: 1, pin: 2
#define ROW_2_PIN			(2)

#define ROW_3_PORT			(1)	//port: 1, pin: 3
#define ROW_3_PIN			(3)


// Masks for the initialization of the KBD controller
#define MASK_P0				(0x4000	| SET_MASK0_FROM_COLUMN(0)                | SET_MASK0_FROM_COLUMN(1)                | SET_MASK0_FROM_COLUMN(2)                | SET_MASK0_FROM_COLUMN(3)                \
									| SET_MASK0_FROM_COLUMN(4)                | SET_MASK0_FROM_COLUMN(5)                | SET_MASK0_FROM_COLUMN(6)                | SET_MASK0_FROM_COLUMN(7) )
uint16_t mask_p0 = MASK_P0;

#define MASK_P12			(0x0000	| SET_MASK12_FROM_COLUMN(0)               | SET_MASK12_FROM_COLUMN(1)               | SET_MASK12_FROM_COLUMN(2)               | SET_MASK12_FROM_COLUMN(3)               \
									| SET_MASK12_FROM_COLUMN(4)               | SET_MASK12_FROM_COLUMN(5)               | SET_MASK12_FROM_COLUMN(6)               | SET_MASK12_FROM_COLUMN(7) )
uint16_t mask_p12 = MASK_P12;

#define MASK_P3				(0x0000	| SET_MASK3_FROM_COLUMN(0)                | SET_MASK3_FROM_COLUMN(1)                | SET_MASK3_FROM_COLUMN(2)                | SET_MASK3_FROM_COLUMN(3)                \
									| SET_MASK3_FROM_COLUMN(4)                | SET_MASK3_FROM_COLUMN(5)                | SET_MASK3_FROM_COLUMN(6)                | SET_MASK3_FROM_COLUMN(7)  )
uint16_t mask_p3 = MASK_P3;

// Masks for the initialization of the WKUP controller
#define WKUP_MASK_P0		(	SET_WKUP_MASK_FROM_COLUMN(0, 0)         | SET_WKUP_MASK_FROM_COLUMN(0, 1)         | SET_WKUP_MASK_FROM_COLUMN(0, 2)         | SET_WKUP_MASK_FROM_COLUMN(0, 3)         \
							  | SET_WKUP_MASK_FROM_COLUMN(0, 4)         | SET_WKUP_MASK_FROM_COLUMN(0, 5)         | SET_WKUP_MASK_FROM_COLUMN(0, 6)         | SET_WKUP_MASK_FROM_COLUMN(0, 7) )
uint16_t wkup_mask_p0 = WKUP_MASK_P0;

#define WKUP_MASK_P1		(	SET_WKUP_MASK_FROM_COLUMN(1, 0)         | SET_WKUP_MASK_FROM_COLUMN(1, 1)         | SET_WKUP_MASK_FROM_COLUMN(1, 2)         | SET_WKUP_MASK_FROM_COLUMN(1, 3)         \
							  | SET_WKUP_MASK_FROM_COLUMN(1, 4)         | SET_WKUP_MASK_FROM_COLUMN(1, 5)         | SET_WKUP_MASK_FROM_COLUMN(1, 6)         | SET_WKUP_MASK_FROM_COLUMN(1, 7) )

uint16_t wkup_mask_p1 = WKUP_MASK_P1;

#define WKUP_MASK_P2		(	SET_WKUP_MASK_FROM_COLUMN(2, 0)         | SET_WKUP_MASK_FROM_COLUMN(2, 1)         | SET_WKUP_MASK_FROM_COLUMN(2, 2)         | SET_WKUP_MASK_FROM_COLUMN(2, 3)         \
							  | SET_WKUP_MASK_FROM_COLUMN(2, 4)         | SET_WKUP_MASK_FROM_COLUMN(2, 5)         | SET_WKUP_MASK_FROM_COLUMN(2, 6)         | SET_WKUP_MASK_FROM_COLUMN(2, 7) )

uint16_t wkup_mask_p2 = WKUP_MASK_P2;

#define WKUP_MASK_P3		(	SET_WKUP_MASK_FROM_COLUMN(3, 0)         | SET_WKUP_MASK_FROM_COLUMN(3, 1)         | SET_WKUP_MASK_FROM_COLUMN(3, 2)         | SET_WKUP_MASK_FROM_COLUMN(3, 3)         \
							  | SET_WKUP_MASK_FROM_COLUMN(3, 4)         | SET_WKUP_MASK_FROM_COLUMN(3, 5)         | SET_WKUP_MASK_FROM_COLUMN(3, 6)         | SET_WKUP_MASK_FROM_COLUMN(3, 7) )

uint16_t wkup_mask_p3 = WKUP_MASK_P3;

KBD_TYPE_QUALIFIER uint8_t kbd_input_ports[] KBD_ARRAY_ATTRIBUTE =
{
	COL(0),                     // column 0 (P0[5])
	COL(1),                     // column 1
	COL(2),                     // column 2
	COL(3),                     // column 3
	COL(4),                     // column 4
	COL(5),                     // column 5
	COL(6),                     // column 6
	COL(7),                     // column 7
};

KBD_TYPE_QUALIFIER uint8_t kbd_output_mode_regs[] KBD_ARRAY_ATTRIBUTE =
{
	SET_OUTPUT_MODE_REG(0),
	SET_OUTPUT_MODE_REG(1),
	SET_OUTPUT_MODE_REG(2),
	SET_OUTPUT_MODE_REG(3),
};

KBD_TYPE_QUALIFIER uint8_t kbd_output_reset_data_regs[] KBD_ARRAY_ATTRIBUTE =
{
	SET_RESET_REG(0),
	SET_RESET_REG(1),
	SET_RESET_REG(2),
	SET_RESET_REG(3),
};

KBD_TYPE_QUALIFIER uint16_t kbd_out_bitmasks[] KBD_ARRAY_ATTRIBUTE =
{
	SET_BITMAP(0),
	SET_BITMAP(1),
	SET_BITMAP(2),
	SET_BITMAP(3),
};

KBD_TYPE_QUALIFIER uint8_t kbd_input_mode_regs[] KBD_ARRAY_ATTRIBUTE =
{
	SET_INPUT_MODE_REG(0),
	SET_INPUT_MODE_REG(1),
	SET_INPUT_MODE_REG(2),
	SET_INPUT_MODE_REG(3),
	SET_INPUT_MODE_REG(4),
	SET_INPUT_MODE_REG(5),
	SET_INPUT_MODE_REG(6),
	SET_INPUT_MODE_REG(7),
};

typedef int kbd_input_ports_check[ (sizeof(kbd_input_ports) / sizeof(uint8_t)) == KBD_NR_INPUTS];                   // on error: the kbd_input_ports[] is not defined properly!
typedef int kbd_output_mode_regs_check[ (sizeof(kbd_output_mode_regs) / sizeof(uint8_t)) == KBD_NR_OUTPUTS];        // on error: the kbd_output_mode_regs[] is not defined properly!
typedef int kbd_output_reset_regs_check[ (sizeof(kbd_output_reset_data_regs) / sizeof(uint8_t)) == KBD_NR_OUTPUTS]; // on error: the kbd_output_reset_data_regs[] is not defined properly!
typedef int kbd_output_bitmasks_check[ (sizeof(kbd_out_bitmasks) / sizeof(uint16_t)) == KBD_NR_OUTPUTS];            // on error: the kbd_out_bitmasks[] is not defined properly!
typedef int kbd_output_input_mode_regs_check[ (sizeof(kbd_input_mode_regs) / sizeof(uint8_t)) == KBD_NR_INPUTS];    // on error: the kbd_input_mode_regs[] is not defined properly!

#ifdef DELAYED_WAKEUP_ON
#define DELAYED_WAKEUP_GPIO_ROW			(X)	// UNDEFINED
#define DELAYED_WAKEUP_GPIO_COLUMN		(Y)	// UNDEFINED
#endif


// extra sets for 'hidden modifiers', e.g. the 'Fn' key
#define KBD_NR_SETS		(2)
#define KBD_IR_NR_SETS	(1)

// unknown key code - nothing is sent to the other side but the key is examined for ghosting
#define K_CODE			(0xF4FF)


// The key map.
// 00xx means regular key
// FCxx means modifier key.
// F8xx means FN Modifier.
// F4xy means special function (x = no of byte in the report, y no of bit set).
KBD_TYPE_QUALIFIER uint16_t kbd_keymap[KBD_NR_SETS][KBD_NR_OUTPUTS][KBD_NR_INPUTS] KBD_ARRAY_ATTRIBUTE =
{
  {
/*    No Fn key(s) pressed
      0         1              2           3         4         5         6         7         
      --------------------------------------------------------------------------------
      Pause     fast fwd    1/2digit       9         6         3         Ok        AV         
      Record    Play       teltxt off      0         8         5         2     	  vol+ 
      Stop      Rewind     teltxt on      shift      7         4         1        Prog-         
      Menu      Vol-         Power       VCR/DVD    Prog+      TV       Mute     ######   
      --------------------------------------------------------------------------------*/     
	{ 0xF402,   0xF404,     0xF416,      0x0026,   0x0023,   0x0020,   0xF411,   0xF417 }, // 0
	{ 0XF403,   0xF401,     0xF4A1,      0x0027,   0x0025,   0x0022,   0x001F,   0xF412 }, // 1
	{ 0xF406,   0xF405,     0xF4A0,      0xFC02,   0x0024,   0x0021,   0x001E,   0xF415 }, // 2
	{ 0xF410,   0xF413,     0xF400,
#if HAS_MULTI_BOND    
                                          PAIR,
#else        
                                         0xF423,  
#endif //HAS_MULTI_BOND
                                                   0xF414,   0xF422,   0xF407,   K_CODE }, // 3
  },
  
//The Report Map (as encoded by the report_map array)
//Byte/Bit      7                6                  5                   4                 3                  2                  1                 0  
//----------------------------------------------------------------------------------------------------------------------------------------------------------
// 0          mute      |	    stop	   |     rewind       |   fast forward   |     record	    |     pause       |	      play	     |      power
// 1    Closed Caption	|      "+10"       |     channel -	  |     channel +    |     volume -	    |    volume +     |	    menu pick    |	     menu
// 2	   {padding}    |     {padding}    |    {padding}     |     {padding}    |     VCR/TV       | media select TV | data on screen_H | data on screen_L 
};

#define KBD_NR_COMBINATIONS 2
// Inputs are active low. Thus, the scan result when both keys are pressed will be 0. 
// In general, for an 1xN matrix, to scan a 2-key combination the following scan status would be expected:
//   0b000...0111...101...101...1
//            ^      ^     ^
//            |      |     |
//            |      |     ---- 2nd key is pressed
//            |      ---- 1st key is pressed
//            ---- highest input (column) of the key matrix (the key is not pressed in this example)
//
// 'action' could be something pre-defined or an EVENT sent to the main FSM to trigger an action
// The priority of key_comb members is decreasing. The first member has the highest priority and it will be checked first!
const struct key_combinations_t key_comb[KBD_NR_COMBINATIONS] =
    {
        {0, 3, 0xF8, 0x1111}, // '' and '' are both pressed
        {1, 3, 0xF8, 0x2222}, // '' and '' are both released
    };


// The IR key map.
// 0x0530 means: 05 is the device address 30 is the device command
KBD_TYPE_QUALIFIER uint16_t kbd_ir_keymap[KBD_IR_NR_SETS][KBD_NR_OUTPUTS][KBD_NR_INPUTS] KBD_ARRAY_ATTRIBUTE =
{
  {
/*    0         1              2           3         4         5         6         7         
      --------------------------------------------------------------------------------
      Pause     fast fwd    1/2digit       9         6         3         Ok        AV         
      Record    Play       teltxt off      0         8         5         2        vol+ 
      Stop      Rewind     teltxt on      shift      7         4         1        Prog-         
      Menu      Vol-         Power       VCR/DVD    Prog+      TV       Mute     ######   
      --------------------------------------------------------------------------------*/     
    { 0x0530,   0x0534,     0x000A,      0x0009,    0x0006,   0x0003,   0x0029,   0x0039 },   // 0
    { 0x0537,   0x0535,     0x003C,      0x0000,    0x0008,   0x0005,   0x0002,   0x0010 },   // 1
    { 0x0536,   0x0532,     0x003C,      0xFFFF,    0x0007,   0x0004,   0x0001,   0x0021 },   // 2
    { 0x0017,   0x0011,     0x000C,      0x0038,    0x0020,   0x002D,   0x000D,   0xFFFF },   // 3
  },
};

/// @} APP

#endif //_APP_KBD_MATRIX_SETUP_8001_H_
